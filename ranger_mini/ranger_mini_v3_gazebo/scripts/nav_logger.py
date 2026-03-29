#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
import datetime

class NavLogger:
    def __init__(self):
        rospy.init_node('nav_logger_node', anonymous=True)
        
        # 状态变量
        self.curr_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.odom_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.ctrl_odom_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.goal_pos = [0.0, 0.0, 0.0] # x, y, yaw
        self.cmd_vel_raw = [0.0, 0.0, 0.0]      # linear_x, linear_y, angular_z
        self.cmd_vel_guarded = [0.0, 0.0, 0.0]  # linear_x, linear_y, angular_z
        self.avg_wheel_speed = 0.0
        self.avg_steer_angle = 0.0
        
        # 订阅话题
        # 1. 订阅 AMCL 提供的地图位姿 (比 odom 更准确)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
        # 1.1 订阅 odom 用于判断底盘是否真实在动
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        # 1.2 控制器自身里程计
        rospy.Subscriber("/four_wheel_steering_controller/odom", Odometry, self.ctrl_odom_cb)
        # 1.3 关节状态（轮速/转向角）
        rospy.Subscriber("/joint_states", JointState, self.joint_cb)
        # 2. 订阅 目标点 (RViz 下发)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        # 3. 订阅 move_base 原始输出与保护后输出
        rospy.Subscriber("/cmd_vel_nav_raw", Twist, self.cmd_raw_cb)
        rospy.Subscriber("/four_wheel_steering_controller/cmd_vel", Twist, self.cmd_guarded_cb)

        # 日志文件路径 (改为 w 模式，每次启动覆盖)
        self.log_file_path = "/tmp/ranger_nav_debug.log"
        rospy.loginfo("[日志节点] 开始记录数据到: " + self.log_file_path)
        
        with open(self.log_file_path, "w") as f:
            f.write("--- 实验开始: {} ---\n".format(datetime.datetime.now()))
            f.write("Time, AMCL_X, AMCL_Y, AMCL_Yaw, Odom_X, Odom_Y, Odom_Yaw, CtrlOdom_X, CtrlOdom_Y, CtrlOdom_Yaw, Goal_X, Goal_Y, Goal_Yaw, Raw_Vx, Raw_Vy, Raw_W, Guard_Vx, Guard_Vy, Guard_W, AvgWheelSpeed, AvgSteerAngle\n")

    def amcl_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.curr_pos = [pos.x, pos.y, yaw]

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.odom_pos = [pos.x, pos.y, yaw]

    def ctrl_odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.ctrl_odom_pos = [pos.x, pos.y, yaw]

    def joint_cb(self, msg):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        name_to_vel = {n: v for n, v in zip(msg.name, msg.velocity)}

        wheel_names = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]
        steer_names = ["fl_steering_joint", "fr_steering_joint", "rl_steering_joint", "rr_steering_joint"]

        wheel_vels = [abs(name_to_vel[n]) for n in wheel_names if n in name_to_vel]
        steer_pos = [abs(name_to_pos[n]) for n in steer_names if n in name_to_pos]

        if wheel_vels:
            self.avg_wheel_speed = sum(wheel_vels) / len(wheel_vels)
        if steer_pos:
            self.avg_steer_angle = sum(steer_pos) / len(steer_pos)

    def goal_cb(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.goal_pos = [pos.x, pos.y, yaw]
        rospy.loginfo("[日志节点] 监听到新目标点!")

    def cmd_raw_cb(self, msg):
        self.cmd_vel_raw = [msg.linear.x, msg.linear.y, msg.angular.z]

    def cmd_guarded_cb(self, msg):
        self.cmd_vel_guarded = [msg.linear.x, msg.linear.y, msg.angular.z]

    def log_status(self):
        timestamp = rospy.get_time()
        log_str = "[Time: {:.2f}] AMCL: [{:.2f}, {:.2f}, {:.2f}] | Odom: [{:.2f}, {:.2f}, {:.2f}] | CtrlOdom: [{:.2f}, {:.2f}, {:.2f}] | Goal: [{:.2f}, {:.2f}, {:.2f}] | RawCmd: [vx:{:.2f}, vy:{:.2f}, w:{:.2f}] | GuardCmd: [vx:{:.2f}, vy:{:.2f}, w:{:.2f}] | Joint: [wheel:{:.3f}, steer:{:.3f}]\n".format(
            timestamp, 
            self.curr_pos[0], self.curr_pos[1], self.curr_pos[2],
            self.odom_pos[0], self.odom_pos[1], self.odom_pos[2],
            self.ctrl_odom_pos[0], self.ctrl_odom_pos[1], self.ctrl_odom_pos[2],
            self.goal_pos[0], self.goal_pos[1], self.goal_pos[2],
            self.cmd_vel_raw[0], self.cmd_vel_raw[1], self.cmd_vel_raw[2],
            self.cmd_vel_guarded[0], self.cmd_vel_guarded[1], self.cmd_vel_guarded[2],
            self.avg_wheel_speed, self.avg_steer_angle
        )
        
        # 实时打印到屏幕
        print(log_str.strip())
        
        # 写入文件
        with open(self.log_file_path, "a") as f:
            f.write(log_str)

    def run(self):
        rate = rospy.Rate(5) # 5Hz 记录频率
        while not rospy.is_shutdown():
            self.log_status()
            rate.sleep()

if __name__ == '__main__':
    try:
        logger = NavLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
