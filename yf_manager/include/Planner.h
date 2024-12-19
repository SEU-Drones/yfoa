#pragma once
#include <iostream>
#include <Eigen/Eigen>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/State.h>

#include "yf_manager/Bspline.h"
#include "yf_manager/WayPoints.h"

#include "InESDFMap.hpp"
#include "HybirdAstar.h"
#include "PathNlopt.h"
#include "UniformBspline.h"

// // 定义颜色代码
// #define RESET   "\033[0m"
// #define RED     "\033[31m"      /* 红色 */
// #define GREEN   "\033[32m"      /* 绿色 */
// #define YELLOW  "\033[33m"      /* 黄色 */
// #define BLUE    "\033[34m"      /* 蓝色 */
// #define MAGENTA "\033[35m"      /* 品红 */
// #define CYAN    "\033[36m"      /* 青色 */
// #define WHITE   "\033[37m"      /* 白色 */

enum FsmState
{
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP
};

struct cameraData
{
    /* depth image process */
    double cx, cy, fx, fy;
    int depth_width, depth_heigth;

    double depth_maxdist, depth_mindist;
    int depth_filter_margin;
    double k_depth_scaling_factor;
    int skip_pixel;

    Eigen::Vector3d camera_pos;
    Eigen::Quaterniond camera_q;

    Eigen::Matrix3d R_C_2_W, R_C_2_B;
    Eigen::Vector3d T_C_2_B, T_C_2_W;

    cv::Mat depth_image;
    pcl::PointCloud<pcl::PointXYZ> ptws_hit, ptws_miss;
    bool has_depth;
};

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

    int traj_id_;
    double duration_;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    ros::Time start_time_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
};

class Planner
{
private:
    PathNlopt::Ptr pathnlopt_ptr_;
    InESDFMap::Ptr workspace_ptr_;
    HybirdAstar::Ptr hybirdastar_ptr_;

    double ctrl_pt_dist_;
    double planning_horizon_;

    MAVTraj trajectory_;
    MAVState end_mavstate_;

    ros::Timer planning_timer_, mapping_timer_;
    ros::Publisher planerflag_pub_, collisionflag_pub_, bspline_pub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
        SyncPolicyImageOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
    SynchronizerImageOdom sync_image_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    ros::Subscriber waypoints_sub_;

    cameraData camData_;

    void setCameraParam(std::string filename);
    void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
    void updateMapCallback(const ros::TimerEvent &);

    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void waypointsCallback(const yf_manager::WayPointsConstPtr &msg);

    bool collisionCheck(double delta, double min_distance);

    double collsion_check_dist_;
    bool callReplan(MAVState start, MAVState end, bool init);
    bool getLocalTarget(MAVState &target, MAVState start, MAVState end, double length);
    void execPlanningCallback(const ros::TimerEvent &e);

    bool have_odom_;
    bool have_target_;
    bool plannerflag_;

    double mapping_time_;

    ros::Publisher new_occ_pub_, new_free_pub_, grid_esdf_pub_;
    ros::Publisher hybird_pub_, optpath_pub_;
    ros::Publisher hybird_pts_pub_, optpath_pts_pub_, smotions_pub_, pts_pub_;

    void publishNewOcc();
    void publishNewFree();
    void publishPath(std::vector<Eigen::Vector3d> path, ros::Publisher pub);
    void publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub);

public:
    Planner(/* args */);
    ~Planner();

    void init(std::string filename, ros::NodeHandle &nh);
};
