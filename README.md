# yfoa
在充满障碍物的空间中，设置若干个途径点，控制无人机安全地依次通过这些点。

# 依赖
源码安装nlopt。（ros提供的存在未知问题）

安装mavros

安装realsense

# 仿真

- 下载代码后catkin_make编译
- 打开程序
    - roslaunch gazebo_simulator mavros_posix_sitl.launch #打开gazebo仿真环境和QGC地面站
    - roslaunch yf_manager yfonboard.launch  # 打开无人机板载程序
    - roslaunch yf_manager yfremotevis.launch # 打开可视化程序
- 通过QGC控制无人机起飞到合适高度，切换offboard模式，观察避障情况。

# 实际飞行
## 起飞前的准备
1、根据飞机的相机，修改config的相机参数；
2、修改onboard.launch中的话题和目标点；

## 起飞
远程登陆无人机的板载计算机，打开如下程序
- 打开mavros，roslaunch yf_manager px4_mavros.launch
- 打开realsense，roslaunch yf_manager rs_depth.launch
- 打开规划，roslaunch yf_manager yfonboard.launch

在地面计算机打开可视化程序（需要配置主从机）：
- roslaunch yf_manager yfremotevis.launch

仔细观察终端和rviz的显示。


# 程序设计
## 约定
- 无人机系统状态：解锁、飞行模式等。
- 无人机状态：无人机位置、姿态及其各阶导数。
- 无人机物理限制：最大速度、最大加速度。
- 无人机轨迹：起点、终点、起始时间、时长、参数轨迹。
- 无人机控制量：设定值、期望值

## mission_node
mission_node控制无人机飞行流程。
- 控制逻辑
    - 如果未解锁或非OFFBOARD，那么复位并退出。
    - 如果当前轨迹存在碰撞风险，那么立即重新规划。
    - 第一次进入OFFBOARD时，对途径点加上Home偏置。
    - 在GEN_NEW_TRAJ、REPLAN_TRAJ、EXEC_TRAJ、WAIT_TRAJ四种状态切换。切换逻辑参考文件state.puml
- 订阅
    - 无人机系统状态
    - 无人机Home点（解锁的位置）
    - 里程计
    - 规划轨迹
    <!-- - 规划结果的标志 to do-->
    - 碰撞检测结果
- 发布
    - 规划器的起点和终点
    - 无人机期望值

## planner_node
planner_node包括建图（gridmap）、碰撞检测和搜索优化（B样条 + hybird A* + nlopt）。规划单独占据一个线程/进程（节点）， 防止规划长时间占用CPU资源。
- 订阅
    - 规划器的起点和终点
- 发布
    - 规划轨迹
    - 碰撞检测结果
