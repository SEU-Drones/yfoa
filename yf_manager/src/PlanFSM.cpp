/*
 * @description:
 * @param:
 * @input:
 * @output:
 */
#include "PlanFSM.h"

PlanFSM::PlanFSM(/* args */)
{
}

PlanFSM::~PlanFSM()
{
}

void PlanFSM::init(std::string filename, ros::NodeHandle &nh)
{
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/depth", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 1));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&PlanFSM::depthOdomCallback, this, _1, _2));

    odometry_sub_ = nh.subscribe("/odom", 1, &PlanFSM::odometryCallback, this);
    waypoints_sub_ = nh.subscribe("/mission/waypoints", 1, &PlanFSM::waypointsCallback, this);
    bspline_pub_ = nh.advertise<yf_manager::Bspline>("/planner/bspline", 10);

    map_timer_ = nh.createTimer(ros::Duration(0.1), &PlanFSM::updateMapCallback, this);
    fsm_timer_ = nh.createTimer(ros::Duration(0.05), &PlanFSM::execFSMCallback, this);

    new_occ_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/new_occ", 10);
    new_free_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/new_free", 10);
    grid_esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/map/grid_esdf", 10);

    hybird_pub_ = nh.advertise<nav_msgs::Path>("/planner/hybird_path", 10);
    optpath_pub_ = nh.advertise<nav_msgs::Path>("/planner/optimal_path", 10);
    hybird_pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planner/hybird_pts", 10);
    optpath_pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planner/optpath_pts", 10);
    pts_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/planner/pts", 10);

    nh.param("mission/control_point_distance", ctrl_pt_dist_, 0.4);
    nh.param("mission/planning_horizon", planning_hor_, 5.0);
    nh.param("mission/no_replan_thresh", no_replan_thresh_, 1.0);
    nh.param("mission/replan_thresh", replan_thresh_, 1.0);

    // 初始化环境
    map_ptr_.reset(new InESDFMap);
    map_ptr_->init(filename);
    setCameraParam(filename);

    // 初始化搜索算法
    hybirdastar_ptr_.reset(new HybirdAstar);
    hybirdastar_ptr_->init(filename, map_ptr_, true);

    // 初始化优化算法
    pathnlopt_ptr_.reset(new PathNlopt);
    pathnlopt_ptr_->init(filename, map_ptr_, true);

    camData_.has_depth = false;
    plan_fsm_state_ = FsmState::INIT;
    have_odom_ = false;
    have_target_ = false;
}

// mapping
void PlanFSM::setCameraParam(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
    }

    cv::FileNode yaml_node = fs["DepthCamera"];
    camData_.depth_heigth = (int)(yaml_node["heigth"]);
    camData_.depth_width = (int)(yaml_node["width"]);
    camData_.fx = (double)(yaml_node["fx"]);
    camData_.fy = (double)(yaml_node["fy"]);
    camData_.cx = (double)(yaml_node["cx"]);
    camData_.cy = (double)(yaml_node["cy"]);

    camData_.k_depth_scaling_factor = (double)(yaml_node["k_depth_scaling_factor"]);
    camData_.depth_maxdist = (double)(yaml_node["depth_maxdist"]);
    camData_.depth_mindist = (double)(yaml_node["depth_mindist"]);
    camData_.depth_filter_margin = (double)(yaml_node["depth_filter_margin"]);
    camData_.skip_pixel = (double)(yaml_node["skip_pixel"]);

    cv::Mat rc2b, tc2b;
    yaml_node["R_C_2_B"] >> rc2b;
    yaml_node["T_C_2_B"] >> tc2b;

    cv::cv2eigen(rc2b, camData_.R_C_2_B);
    cv::cv2eigen(tc2b, camData_.T_C_2_B);

    // camData_.R_C_2_B << 0, 0, 1, -1, 0, 0, 0, -1, 0; // realsense
    // // camData_.R_C_2_B << 0, 0, 1, 0, -1, 0, 1, 0, 0;//竖着放置 for sg
    // camData_.T_C_2_B << 0.0, 0.0, 0.0;

    std::cout << "[CameraParam INIT] use depth camera" << std::endl;
    std::cout << "[CameraParam INIT] depth heigth: " << camData_.depth_heigth << std::endl;
    std::cout << "[CameraParam INIT] depth width: " << camData_.depth_width << std::endl;
    std::cout << "[CameraParam INIT] depth fx: " << camData_.fx << std::endl;
    std::cout << "[CameraParam INIT] depth fy: " << camData_.fy << std::endl;
    std::cout << "[CameraParam INIT] depth cx: " << camData_.cx << std::endl;
    std::cout << "[CameraParam INIT] depth cy: " << camData_.cy << std::endl;
    std::cout << "[CameraParam INIT] depth k_depth_scaling_factor: " << camData_.k_depth_scaling_factor << std::endl;
    std::cout << "[CameraParam INIT] depth depth_maxdist: " << camData_.depth_maxdist << std::endl;
    std::cout << "[CameraParam INIT] depth depth_mindist: " << camData_.depth_mindist << std::endl;
    std::cout << "[CameraParam INIT] depth depth_filter_margin: " << camData_.depth_filter_margin << std::endl;
    std::cout << "[CameraParam INIT] depth skip_pixel: " << camData_.skip_pixel << std::endl;
    std::cout << "[CameraParam INIT] R_C_2_B: \n"
              << camData_.R_C_2_B << std::endl;
    std::cout << "[CameraParam INIT] T_C_2_B: " << camData_.T_C_2_B.transpose() << std::endl;
}

void PlanFSM::depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom)
{
    camData_.camera_pos(0) = odom->pose.pose.position.x;
    camData_.camera_pos(1) = odom->pose.pose.position.y;
    camData_.camera_pos(2) = odom->pose.pose.position.z;
    camData_.camera_q = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                           odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, camData_.k_depth_scaling_factor);
    }
    cv_ptr->image.copyTo(camData_.depth_image);

    camData_.has_depth = true;
}

void PlanFSM::updateMapCallback(const ros::TimerEvent &)
{
    if (camData_.has_depth != true)
        return;

    camData_.has_depth = false;

    camData_.R_C_2_W = camData_.camera_q.toRotationMatrix() * camData_.R_C_2_B;
    camData_.T_C_2_W = camData_.camera_pos + camData_.T_C_2_B;
    camData_.ptws_hit.clear();
    camData_.ptws_miss.clear();

    Eigen::Vector3d pt_w;
    pcl::PointXYZ pt;
    double depth;

    uint16_t *row_ptr;
    int cols = camData_.depth_image.cols;
    int rows = camData_.depth_image.rows;

    const double inv_factor = 1.0 / camData_.k_depth_scaling_factor;

    // if (true)
    // {
    //     local_map_boundary_min_ = camData_.camera_pos;
    //     local_map_boundary_max_ = camData_.camera_pos;
    // }

    for (int v = camData_.depth_filter_margin; v < rows - camData_.depth_filter_margin; v += camData_.skip_pixel)
    {
        row_ptr = camData_.depth_image.ptr<uint16_t>(v) + camData_.depth_filter_margin;

        for (int u = camData_.depth_filter_margin; u < cols - camData_.depth_filter_margin; u += camData_.skip_pixel)
        {
            depth = (*row_ptr) * inv_factor;
            row_ptr = row_ptr + camData_.skip_pixel;

            if (*row_ptr == 0 || depth > camData_.depth_maxdist)
            {
                depth = camData_.depth_maxdist;

                pt_w(0) = (u - camData_.cx) * depth / camData_.fx;
                pt_w(1) = (v - camData_.cy) * depth / camData_.fy;
                pt_w(2) = depth;
                pt_w = camData_.R_C_2_W * pt_w + camData_.T_C_2_W;

                pt.x = pt_w(0);
                pt.y = pt_w(1);
                pt.z = pt_w(2);

                camData_.ptws_miss.points.push_back(pt);
            }
            else if (depth < camData_.depth_mindist)
            {
                continue;
            }
            else
            {
                pt_w(0) = (u - camData_.cx) * depth / camData_.fx;
                pt_w(1) = (v - camData_.cy) * depth / camData_.fy;
                pt_w(2) = depth;
                pt_w = camData_.R_C_2_W * pt_w + camData_.T_C_2_W;

                pt.x = pt_w(0);
                pt.y = pt_w(1);
                pt.z = pt_w(2);

                // if (pt.z < -1.0)
                //     continue;

                camData_.ptws_hit.points.push_back(pt);
            }

            // if (true)
            // {
            //     local_map_boundary_max_(0) = std::max(local_map_boundary_max_(0), pt_w(0));
            //     local_map_boundary_max_(1) = std::max(local_map_boundary_max_(1), pt_w(1));
            //     local_map_boundary_max_(2) = std::max(local_map_boundary_max_(2), pt_w(2));

            //     local_map_boundary_min_(0) = std::min(local_map_boundary_min_(0), pt_w(0));
            //     local_map_boundary_min_(1) = std::min(local_map_boundary_min_(1), pt_w(1));
            //     local_map_boundary_min_(2) = std::min(local_map_boundary_min_(2), pt_w(2));
            // }
        }
    }

    // ros::Time t1, t2;
    // t1 = ros::Time::now();
    map_ptr_->update(&camData_.ptws_hit, &camData_.ptws_miss, camData_.camera_pos);
    // t2 = ros::Time::now();
    // std::cout << "esdf update: " << ros::Time::now() << "    " << (t2 - t1).toSec() << std::endl;

    publishNewOcc();
    publishNewFree();

    static tf::TransformBroadcaster br;
    Eigen::Quaterniond eq(camData_.R_C_2_W);
    br.sendTransform(tf::StampedTransform(tf::Transform(
                                              tf::Quaternion(eq.w(), eq.x(), eq.y(), eq.z()),
                                              tf::Vector3(camData_.T_C_2_W(0), camData_.T_C_2_W(1), camData_.T_C_2_W(2))),
                                          ros::Time::now(), "map", "base_link"));
}

// // planning
// void PlanFSM::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
// {
//     trajectory_.start_time_ = time_now;
//     trajectory_.position_traj_ = position_traj;
//     trajectory_.velocity_traj_ = trajectory_.position_traj_.getDerivative();
//     trajectory_.acceleration_traj_ = trajectory_.velocity_traj_.getDerivative();
//     trajectory_.start_pos_ = trajectory_.position_traj_.evaluateDeBoorT(0.0);
//     trajectory_.duration_ = trajectory_.position_traj_.getTimeSum();
//     trajectory_.traj_id_ += 1;
// }

bool PlanFSM::callReplan(MAVState start, MAVState end, bool init)
{
    double time_interval;
    pathnlopt_ptr_->setPhysicLimits(trajectory_.max_vel, trajectory_.max_acc);
    hybirdastar_ptr_->setPhysicLimits(trajectory_.max_vel, trajectory_.max_acc);
    time_interval = ctrl_pt_dist_ / trajectory_.max_vel;
    // time_interval = 0.1;

    pathnlopt_ptr_->setTimeInterval(time_interval);

    std::vector<Eigen::Vector3d> search_path, opt_path;
    Eigen::MatrixXd opt_var;

    hybirdastar_ptr_->search(start.pos, start.vel, start.acc, end.pos, end.vel, init, planning_hor_);

    search_path = hybirdastar_ptr_->getKinoTraj(time_interval);

    std::cout<<ros::Time::now().toSec()<<" hy input start: "<<start.pos.transpose()<<std::endl;
    std::cout<<ros::Time::now().toSec()<<" hy output start: "<<search_path[0].transpose()<<std::endl;

    if (search_path.size() < 1)
        return false;

    publishPath(search_path, hybird_pub_);
    publishPoints(search_path, hybird_pts_pub_);

    opt_var.resize(3, search_path.size());
    for (int i = 0; i < search_path.size(); i++)
        opt_var.col(i) = search_path[i];

    pathnlopt_ptr_->setOptVar(opt_var);
    pathnlopt_ptr_->optimize();

    opt_path = pathnlopt_ptr_->getOptimizeTraj();

    std::cout<<ros::Time::now().toSec()<<" opt output start: "<<opt_path[0].transpose()<<std::endl;


    publishPath(opt_path, optpath_pub_);
    publishPoints(search_path, optpath_pts_pub_);

    // updateTrajInfo(UniformBspline(pathnlopt_ptr_->getMatrixOptimizeTraj(), 3, time_interval), ros::Time::now());
    // trajectory_.start_time_ = ros::Time::now();
    trajectory_.position_traj_ = UniformBspline(pathnlopt_ptr_->getMatrixOptimizeTraj(), 3, time_interval);
    trajectory_.velocity_traj_ = trajectory_.position_traj_.getDerivative();
    trajectory_.acceleration_traj_ = trajectory_.velocity_traj_.getDerivative();
    trajectory_.start_pos_ = trajectory_.position_traj_.evaluateDeBoorT(0.0);
    trajectory_.duration_ = trajectory_.position_traj_.getTimeSum();
    
    trajectory_.traj_id_ += 1;

    // MAVTraj *info = &trajectory_;

    yf_manager::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = trajectory_.start_time_;
    bspline.traj_id = trajectory_.traj_id_;

    Eigen::MatrixXd pos_pts = trajectory_.position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = trajectory_.position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
        bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
}

void PlanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    current_mavstate_.pos(0) = msg->pose.pose.position.x;
    current_mavstate_.pos(1) = msg->pose.pose.position.y;
    current_mavstate_.pos(2) = msg->pose.pose.position.z;

    current_mavstate_.vel(0) = msg->twist.twist.linear.x;
    current_mavstate_.vel(1) = msg->twist.twist.linear.y;
    current_mavstate_.vel(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    current_mavstate_.quat.w() = msg->pose.pose.orientation.w;
    current_mavstate_.quat.x() = msg->pose.pose.orientation.x;
    current_mavstate_.quat.y() = msg->pose.pose.orientation.y;
    current_mavstate_.quat.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
}

void PlanFSM::waypointsCallback(const yf_manager::WayPointsConstPtr &msg)
{
    target_mavstate_.pos << msg->pos_x[0], msg->pos_y[0], msg->pos_z[0];
    target_mavstate_.vel.setZero();
    target_mavstate_.acc.setZero();

    trajectory_.max_vel = msg->max_vel[0];
    trajectory_.max_acc = msg->max_acc[0];
    have_target_ = true;
}

void PlanFSM::changeFSMExecState(FsmState new_state, string pos_call)
{

    // if (new_state == exec_state_)
    //   continously_called_times_++;
    // else
    //   continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(plan_fsm_state_);
    plan_fsm_state_ = new_state;
    std::cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << std::endl;
}

void PlanFSM::getLocalTarget(MAVState &target, MAVState cur, MAVState end, double length)
{
    length = 1.5 * length;
    // std::cout << "getLocaltarget" << length << std::endl;
    double distance = (end.pos - cur.pos).norm();
    if (distance < length)
        target = end;
    else
    {
        target.pos = cur.pos + (end.pos - cur.pos) * (length / distance);
        target.vel.setZero();
        target.acc.setZero();
    }
}

void PlanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    switch (plan_fsm_state_)
    {
    case FsmState::INIT:
    {
        if (!have_odom_)
        {
            return;
        }
        // if (!trigger_)
        // {
        //     return;
        // }
        changeFSMExecState(FsmState::WAIT_TARGET, "FSM");

        break;
    }
    case FsmState::WAIT_TARGET:
    {
        if (!have_target_)
            return;
        else
        {
            changeFSMExecState(FsmState::GEN_NEW_TRAJ, "FSM");
        }
        break;
    }
    case FsmState::GEN_NEW_TRAJ:
    {
        // 获取当前期望的目标点
        // 目标是否处在障碍物内部
        trajectory_.start_mavstate = current_mavstate_;
        trajectory_.start_time_ = ros::Time::now();

        getLocalTarget(trajectory_.end_mavstate, trajectory_.start_mavstate, target_mavstate_, planning_hor_);

        bool success = callReplan(trajectory_.start_mavstate, trajectory_.end_mavstate, false);

        if (success)
            changeFSMExecState(FsmState::EXEC_TRAJ, "FSM");
        else
            changeFSMExecState(FsmState::GEN_NEW_TRAJ, "FSM");

        break;
    }
    case FsmState::EXEC_TRAJ:
    {
        ros::Time time_now = ros::Time::now();

        double t_cur = (time_now - trajectory_.start_time_).toSec();
        t_cur = std::min(trajectory_.duration_, t_cur);

        Eigen::Vector3d pos = trajectory_.position_traj_.evaluateDeBoorT(t_cur);

        /* && (end_pt_ - pos).norm() < 0.5 */
        if (t_cur > trajectory_.duration_ - 1e-2) // 轨迹结束
        {
            have_target_ = false;

            changeFSMExecState(FsmState::WAIT_TARGET, "FSM");
            return;
        }
        else if ((target_mavstate_.pos - pos).norm() < no_replan_thresh_) // 到达终点附近，不再规划
        {
            // cout << "near end" << endl;
            return;
        }
        else if ((trajectory_.start_pos_ - pos).norm() < replan_thresh_)
        {
            // cout << "near start" << endl;
            return;
        }
        else
        {
            changeFSMExecState(FsmState::REPLAN_TRAJ, "FSM");
        }
        break;
    }

    case FsmState::REPLAN_TRAJ:
    {
        // 获取当前期望的目标点
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - trajectory_.start_time_).toSec();

        trajectory_.start_mavstate.pos = trajectory_.position_traj_.evaluateDeBoorT(t_cur);
        trajectory_.start_mavstate.vel = trajectory_.velocity_traj_.evaluateDeBoorT(t_cur);
        trajectory_.start_mavstate.acc = trajectory_.acceleration_traj_.evaluateDeBoorT(t_cur);
        trajectory_.start_time_ = ros::Time::now();

        std::cout <<ros::Time::now().toSec()<<" start "<< t_cur << " " << trajectory_.start_mavstate.pos.transpose() << std::endl;

        getLocalTarget(trajectory_.end_mavstate, trajectory_.start_mavstate, target_mavstate_, planning_hor_);
        // std::cout << trajectory_.start_mavstate.pos.transpose() << "  " << target_mavstate_.pos.transpose() << std::endl;

        bool success = callReplan(trajectory_.start_mavstate, trajectory_.end_mavstate, true);

        std::cout <<ros::Time::now().toSec()<< " 0.0 " << trajectory_.position_traj_.evaluateDeBoorT(0.0).transpose() << std::endl;
        std::cout <<ros::Time::now().toSec()<<" "<< (ros::Time::now() - trajectory_.start_time_).toSec() << " " << trajectory_.position_traj_.evaluateDeBoorT((ros::Time::now() - trajectory_.start_time_).toSec()).transpose() << std::endl;

        std::vector<Eigen::Vector3d> points;
        points.push_back(trajectory_.start_mavstate.pos);
        publishPoints(points, pts_pub_);

        if (success)
            changeFSMExecState(FsmState::EXEC_TRAJ, "FSM");
        else
            changeFSMExecState(FsmState::GEN_NEW_TRAJ, "FSM");
        break;
    }
    case FsmState::EMERGENCY_STOP:
    {
        break;
    }
    }
}

// 可视化
void PlanFSM::publishNewOcc()
{
    std::vector<int> *newOcc;
    newOcc = map_ptr_->SOGMPtr_->getNewOcc();

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;

    for (int i = 0; i < newOcc->size(); i++)
    {
        pos = map_ptr_->SOGMPtr_->IndexToWorld(newOcc->at(i));

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    new_occ_pub_.publish(cloud_msg);
}

void PlanFSM::publishNewFree()
{
    std::vector<int> *newFree;
    newFree = map_ptr_->SOGMPtr_->getNewFree();

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3d pos;

    for (int i = 0; i < newFree->size(); i++)
    {
        pos = map_ptr_->SOGMPtr_->IndexToWorld(newFree->at(i));

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    new_free_pub_.publish(cloud_msg);
}

void PlanFSM::publishPath(std::vector<Eigen::Vector3d> path, ros::Publisher pub)
{
    nav_msgs::Path path_ros;
    geometry_msgs::PoseStamped pt;
    for (int i = 0; i < path.size(); i++)
    {
        pt.pose.position.x = path[i](0);
        pt.pose.position.y = path[i](1);
        pt.pose.position.z = path[i](2);
        path_ros.poses.push_back(pt);
    }
    path_ros.header.frame_id = "map";
    pub.publish(path_ros);
}

void PlanFSM::publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub)
{
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < points.size(); i++)
    {
        pt.x = points[i](0);
        pt.y = points[i](1);
        pt.z = points[i](2);
        cloud.points.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "map";

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    pub.publish(cloud_msg);
}
