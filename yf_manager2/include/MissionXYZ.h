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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "yf_manager2/WayPoints.h"
#include "yf_manager2/Bspline.h"

#include "UniformBspline.h"

struct Point
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    double longitude; // 经度
    double latitude;  // 纬度
    double yaw;
    double yaw_dot;
};

enum ControlMode
{
    POS = 0,
    POSYAW = 1,
    POSVELYAW = 2,
    POSVELACCYAW = 3,
    VEL
};

class MissionXYZ
{
private:
    std::vector<Point> wps_;
    int wps_num_; // 经过点集合
    double wps_thr_;

    Point home_; // 起点位置

    mavros_msgs::State state_;
    nav_msgs::Odometry odom_;

    int mission_fsm_state_; // 控制状态切换
    enum MISSION_STATE
    {
        IDLE = 0,
        READY,
        TAKEOFF,
        MOVE,
        LAND
    };

    ros::Timer mission_fsm_timer_, cmd_timer_;
    ros::Publisher wps_pub_, setpoint_raw_local_pub_;
    ros::Subscriber state_sub_, odom_sub_, rviz_sub_, bspline_sub_;

    std::vector<Eigen::Vector3d> pos_cmds_, pos_actual_;
    ros::Publisher poscmds_vis_pub_, posactual_vis_pub_;

    bool has_odom_;
    bool sendOneByOne_;
    bool sendflag_;
    int k_;

    int control_mode_;
    bool receive_traj_;
    std::vector<UniformBspline> traj_;
    double traj_duration_;
    ros::Time start_time_;
    int traj_id_;

    Eigen::Vector3d pos_sp_, vel_sp_, acc_sp_; // 定义成全局变量，保证记录最后一个点
    double yaw_sp_;
    double last_yaw_, last_yaw_dot_;
    double time_forward_;

    std::string handle_wpts_xy_,handle_wpts_z_;

    void setHome(nav_msgs::Odometry odom, Point &home);
    void sendWayPoints(std::vector<Point> wayPoints);
    void sendWayPoints(std::vector<Point> wayPoints, int k);
    void changeMissionState(int &mode, int next);
    void missionCallback(const ros::TimerEvent &e); // Timer for workflow control, send WayPoints

    void stateCallback(const mavros_msgs::State::ConstPtr &msg); // subscribe the mav flight mode
    void bsplineCallback(yf_manager2::BsplineConstPtr msg);
    void localOdomCallback(const nav_msgs::OdometryConstPtr &msg); // subscribe the mav odom
    void rvizCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void sendCmd(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, int cmode);
    void cmdCallback(const ros::TimerEvent &e);

    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last);
    double quaternion_to_yaw(geometry_msgs::Quaternion &q);
    void publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub);

public:
    MissionXYZ();
    ~MissionXYZ();

    void init(ros::NodeHandle node); //  初始化
};

#endif