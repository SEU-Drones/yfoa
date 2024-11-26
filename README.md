# yfoa
避障程序。

# 使用
源码安装nlopt。（ros提供的存在未知问题）

安装mavros

安装realsense

# 仿真

下载代码后catkin_make编译

- 打开gazebo仿真环境和QGC地面站

roslaunch gazebo_simulator mavros_posix_sitl.launch 


- 打开无人机板载程序

roslaunch yf_manager yfonboard.launch 

- 打开可视化程序

roslaunch yf_manager yfremotevis.launch

- 通过QGC控制无人机起飞到合适高度，切换offboard模式，观察避障情况。

# 实际飞行

## 主从机