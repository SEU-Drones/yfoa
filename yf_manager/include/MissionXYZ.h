/**
 * @Name: MissionSample
 * @Author: Yong
 * @Version: 1.0
 * @Date: 2022-10-25 20:16:47
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-06-29 20:23:03
 * @description:  Overall workflow control by sending WayPoints.
 * @input: mav state & mav odom & rviz setpoint & pre-determined WayPoints
 * @output: WayPoints
 */
#ifndef MISSION_XYZ_H
#define MISSION_XYZ_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Eigen>
#include <cmath>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>

#include "yf_manager/WayPoints.h"
#include "yf_manager/Bspline.h"

#include "UniformBspline.h"

struct MAVState
{
    ros::Time time;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;

    Eigen::Quaterniond quat;

    double yaw;
    double yaw_dot;

    double longitude; // 经度
    double latitude;  // 纬度
};
struct MAVLimits
{
    double max_vel;
    double max_acc;
};

struct MAVTraj
{
    std::vector<MAVState> waypints;

    double max_vel;
    double max_acc;

    // 局部的轨迹
    MAVState start_mavstate, end_mavstate;

    int id;
    double duration;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    ros::Time start_time;
    UniformBspline position_traj, velocity_traj, acceleration_traj;
};
class MissionXYZ
{
private:
    enum ControlMode
    {
        POS = 0,
        POSYAW = 1,
        POSVELYAW = 2,
        POSVELACCYAW = 3,
        VEL
    };
    int mission_fsm_state_; // 控制状态切换
    enum MISSION_STATE
    {
        READY = 0,
        MANUALFLIGHT,
        AUTOFLIGHT,
    };
    int mission_auto_fsm_state_; // 控制状态切换
    enum MISSION_AUTO_STATE
    {
        GEN_NEW_TRAJ,
        EXEC_TRAJ,
        REPLAN_TRAJ
    };

    mavros_msgs::State uav_sysstate_, last_uav_sysstate_;
    // nav_msgs::Odometry odom_;

    std::vector<MAVState> wps_; // 途径点
    int wps_num_;

    MAVState home_; // home位置
    MAVState current_state_, end_state_;

    int control_mode_; // 规划轨迹
    bool receive_traj_;
    MAVTraj trajectory_;

    double time_forward_;

    double wps_thr_; // 判断到达途径点阈值
    std::string handle_wpts_xy_, handle_wpts_z_;

    double no_replan_thresh_, replan_thresh_;

    bool sendOneByOne_;
    bool sendflag_;
    int k_;
    std::ofstream yf_mission_file_;

    bool has_odom_, has_depth_;
    bool collision_;

    ros::Timer mission_fsm_timer_;
    void changeMissionState(int &mode, int next);
    // void setHome(nav_msgs::Odometry odom, MAVState &home);
    void publishCmd(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, int cmode);
    void missionCallback(const ros::TimerEvent &e); // Timer for workflow control, send WayPoints

    void stateCallback(const mavros_msgs::State::ConstPtr &msg); // subscribe the mav flight mode
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);    // subscribe the mav odom
    void planerFlagCallback(const std_msgs::Int16 &msg);
    void bsplineCallback(yf_manager::BsplineConstPtr msg);
    void rvizCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void publishSE(MAVState start, MAVState end);

    ros::Publisher wps_pub_, setpoint_raw_local_pub_;
    ros::Subscriber state_sub_, odom_sub_, rviz_sub_, bspline_sub_, planerflag_sub_;

    std::vector<Eigen::Vector3d> pos_cmds_, pos_actual_;
    ros::Publisher poscmds_vis_pub_, posactual_vis_pub_;

    Eigen::Vector3d pos_sp_, vel_sp_, acc_sp_; // 定义成全局变量，保证记录最后一个点
    double yaw_sp_;
    double last_yaw_, last_yaw_dot_;

    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last);
    double quaternion_to_yaw(geometry_msgs::Quaternion &q);
    void publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub);

public:
    MissionXYZ();
    ~MissionXYZ();

    void init(ros::NodeHandle node); //  初始化
};

#endif