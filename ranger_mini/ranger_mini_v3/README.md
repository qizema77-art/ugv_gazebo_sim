# Ranger Mini V3 仿真说明文档

本文档包含了在 Gazebo 仿真环境中使用 Ranger Mini V3 机器人进行 **SLAM (同步定位与建图)** 和 **Navigation (自主导航)** 的完整工作流与操作指令。

---

## 1. 启动SLAM建图 (Gmapping)

通过雷达扫描周边环境，配合北通鲲鹏20手柄遥控小车，走遍仿真场景以构建二维地图。

### 步骤 1：启动仿真环境与 Gmapping 节点
打开终端1，运行建图 launch 文件（已包含小车生成及 SLAM 算法）：
```bash
roslaunch ranger_mini_v3_gazebo ranger_gmapping.launch
```

### 步骤 2：启动 RViz 观察建图进度
打开终端2，运行 RViz 文件进行可视化监控：
```bash
roslaunch ranger_mini_v3_gazebo ranger_rviz.launch
```
*(在 RViz 中，确保 Fixed Frame 设置为 `map`，并添加 Map 与 LaserScan 显示)*

### 步骤 3：启动手柄遥控控制
打开终端3，连接手柄后运行遥控节点：
```bash
roslaunch ranger_mini_v3_gazebo teleop_btp20.launch
```
**操作方法**：
- 按住（长按） **A 键 (Button 0)** 激活操作（死人开关）。
- 左摇杆推拉控制 **前进 / 后退**。
- 左摇杆左右控制 **左转 / 右转**。
遥控小车在场景里缓慢绕场一周，观察 RViz 里的地图直至完整闭合。

### 步骤 4：保存地图
当建图完成后，**不要关闭终端1和终端2！**
打开终端4，使用 `map_saver` 保存当前地图（它会生成 my_map.yaml 和 my_map.pgm 两个文件）：
```bash
rosrun map_server map_saver -f ~/ranger_ws/src/ugv_gazebo_sim/ranger_mini/ranger_mini_v3_gazebo/maps/my_map
```

---

## 2. 启动自主导航 (Navigation)

使用上一步保存的地图，利用 AMCL 进行定位，并在 RViz 中指派目标点，让小车自主规划路线前行。

*(提示：运行本环节前，请先关闭之前建图打开的所有相关终端！)*

### 步骤 1：启动导航堆栈
打开终端1，运行包含 Gazebo 环境、AMCL 定位与 MoveBase 的全局文件：
```bash
roslaunch ranger_mini_v3_gazebo ranger_navigation.launch
```

### 步骤 2：打开 RViz 进行交互
打开终端2，启动 RViz：
```bash
roslaunch ranger_mini_v3_gazebo ranger_rviz.launch
```

### 步骤 3：初始化小车位姿 (AMCL 匹配)
在 RViz 中：
1. 确保左侧的 **Fixed Frame** 选项已经选择为 `map`。
2. 点击顶部工具栏的 **"2D Pose Estimate"** 按钮（绿色粗箭头）。
3. 参照 Gazebo 里的画面，在 RViz 地图中的对应位置**点击左键并顺着车头方向拖动一段距离再松开**。
4. 你会看到满屏的绿色点云（Particle Cloud）瞬间匹配到小车的真实位置，雷达红点与墙壁轮廓对齐。

*(可选)：如果你不想手动指定位置，可以使用全局自动撒点功能：打开新终端执行 `rosservice call /global_localization "{}"`，然后再用极慢的速度用手柄开一会儿车，车会自动找到自己在地图上的位置。*

### 步骤 4：发送导航目标 (Nav Goal)
位姿校准完毕后：
1. 点击 RViz 顶部工具栏界面的 **"2D Nav Goal"** 按钮。
2. 在地图的空旷路面上，点击并拖动设定小车将要开往的目的地和最终停车朝向。
3. 系统将会规划出一条从当前位置到目标的路径（通常是一条长线），随后小车（作为阿克曼结构车辆）会自动倒车微调或者边走边转开往目的地。

---

*Enjoy the ride! 🚗💨*
