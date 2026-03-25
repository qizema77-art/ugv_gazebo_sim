#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class CustomAckermannPlanner:
    def __init__(self):
        rospy.init_node("custom_ackermann_planner", anonymous=True)

        rospy.loginfo("[自定义规划器] 正在初始化...")

        # 订阅与发布
        self.cmd_pub = rospy.Publisher("/four_wheel_steering_controller/cmd_vel", Twist, queue_size=1)
        
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/scan", LaserScan, self.scan_cb)

        # 状态变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.goal_x = None
        self.goal_y = None
        
        self.obstacle_distance_front = 999.0
        
        # 控制器参数 (阿克曼)
        self.max_speed = 0.8
        self.min_speed = 0.05
        self.max_steer = 0.4 # 设定为用户请求的 0.4rad/s
        
        self.goal_tolerance = 0.2 # 距离误差
        self.is_navigating = False

        rospy.loginfo("[自定义规划器] 初始化完成！等待RViz中下发 '2D Nav Goal'...")

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_cb(self, msg):
        """局部避障: 检测正前方障碍物距离"""
        # 雷达数据中间部分代表正前方 (假设180度或360度扫描)
        ranges = msg.ranges
        num_ranges = len(ranges)
        
        # 取中间30度的视野（大概前方15度到-15度）
        angle_view = math.radians(30.0)
        angle_increment = msg.angle_increment
        indices_to_check = int((angle_view / 2) / angle_increment)
        
        center_index = num_ranges // 2 # 假设正中心在数组中间
        start_idx = max(0, center_index - indices_to_check)
        end_idx = min(num_ranges, center_index + indices_to_check)
        
        front_ranges = [r for r in ranges[start_idx:end_idx] if r > msg.range_min and not math.isinf(r) and not math.isnan(r)]
        
        if front_ranges:
            self.obstacle_distance_front = min(front_ranges)
        else:
            self.obstacle_distance_front = 999.0

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        rospy.loginfo("[全局规划] 收到新目标点！ X: {:.2f}, Y: {:.2f}".format(self.goal_x, self.goal_y))
        
        # 这里为了简化和测试，我们不生成A*全局路径，而是把目标作为直接引路点 (Pure Pursuit 简化版)
        self.is_navigating = True

    def calculate_control(self):
        """局部规划与控制计算"""
        if not self.is_navigating or self.goal_x is None:
            return

        cmd = Twist()
        
        # 1. 计算到目标的距离和角度偏差
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.hypot(dx, dy)
        
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_yaw
        
        # 规范化角度差至 [-pi, pi]
        while angle_diff > math.pi:  angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi: angle_diff += 2.0 * math.pi
        
        rospy.loginfo("[局部规划] 距目标: {:.2f}m, 角度偏差: {:.2f}rad, 前方障碍距离: {:.2f}m".format(
            distance_to_goal, angle_diff, self.obstacle_distance_front
        ))

        # 2. 判断是否到达目标点
        if distance_to_goal < self.goal_tolerance:
            rospy.loginfo("[追踪完成] 到达目标点附近，停车！")
            self.is_navigating = False
            self.cmd_pub.publish(cmd) # 发送全0停车
            return

        # 3. 局部避障逻辑 (非常简单的防撞)
        if self.obstacle_distance_front < 0.6:
            rospy.logwarn("[局部避障] 前方 {:.2f}m 处有障碍物！启动停车或避障角！".format(self.obstacle_distance_front))
            # 简单避障：后退或极限打方向转行
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_steer # 原地转个方向(如果是阿克曼，后退转弯更有效)
            self.cmd_pub.publish(cmd)
            return

        # 4. 阿克曼运动学计算 (计算线速度和转向角速度)
        # 前进速度
        speed = min(self.max_speed, distance_to_goal)
        
        # 为了符合阿克曼，在角度偏差很大的时候需要减速甚至倒车来进行回转
        if abs(angle_diff) > math.pi / 2: # 目标在后面
            speed = -self.max_speed * 0.5 # 倒车
            rospy.loginfo("[控制] 目标在后方，执行倒车对齐。")
            steering = self.max_steer if angle_diff > 0 else -self.max_steer

        else: # 目标在前方
            # 根据距离逐渐减速
            if distance_to_goal < 1.0:
                speed = max(self.min_speed, speed * 0.5)
            
            # 使用比例控制打方向盘
            Kp_steer = 1.0 # 转向比例系数
            steering = Kp_steer * angle_diff
            
            # 限制最大转向角
            steering = max(min(steering, self.max_steer), -self.max_steer)

        cmd.linear.x = speed
        cmd.angular.z = steering
        
        rospy.loginfo("[控制指令下发] 线速度: {:.2f} m/s, 角速度: {:.2f} rad/s".format(cmd.linear.x, cmd.angular.z))
        self.cmd_pub.publish(cmd)

    def run(self):
        rate = rospy.Rate(10) # 10Hz 控制环
        while not rospy.is_shutdown():
            self.calculate_control()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = CustomAckermannPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
