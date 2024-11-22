#pragma once
#include <iostream>
#include <Eigen/Eigen>

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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include "yf_manager/Bspline.h"
#include "yf_manager/WayPoints.h"

#include "InESDFMap.hpp"
#include "HybirdAstar.h"
#include "PathNlopt.h"
#include "UniformBspline.h"

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
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;

    Eigen::Quaterniond quat;
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
    Eigen::Vector3d start_pos_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
};

class PlanFSM
{
private:
    PathNlopt::Ptr pathnlopt_ptr_;
    InESDFMap::Ptr map_ptr_;
    HybirdAstar::Ptr hybirdastar_ptr_;

    double ctrl_pt_dist_;
    double planning_hor_;

    MAVTraj trajectory_;

    ros::Timer fsm_timer_, map_timer_;
    ros::Publisher bspline_pub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
        SyncPolicyImageOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
    SynchronizerImageOdom sync_image_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

    ros::Subscriber waypoints_sub_, odometry_sub_;

    cameraData camData_;

    MAVState current_mavstate_, target_mavstate_;

    void setCameraParam(std::string filename);
    void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
    void updateMapCallback(const ros::TimerEvent &);

    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void waypointsCallback(const yf_manager::WayPointsConstPtr &msg);

    double no_replan_thresh_, replan_thresh_;
    FsmState plan_fsm_state_;
    void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);
    bool callReplan(MAVState start, MAVState end, bool init);
    void changeFSMExecState(FsmState new_state, string pos_call);
    void getLocalTarget(MAVState &target, MAVState cur, MAVState end, double length);
    void execFSMCallback(const ros::TimerEvent &e);

    bool have_odom_;
    bool have_target_;

    ros::Publisher new_occ_pub_, new_free_pub_, grid_esdf_pub_;
    ros::Publisher hybird_pub_, optpath_pub_;
    ros::Publisher hybird_pts_pub_, optpath_pts_pub_, smotions_pub_, pts_pub_;

    void publishNewOcc();
    void publishNewFree();
    void publishPath(std::vector<Eigen::Vector3d> path, ros::Publisher pub);
    void publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub);

public:
    PlanFSM(/* args */);
    ~PlanFSM();

    void init(std::string filename, ros::NodeHandle &nh);
};
