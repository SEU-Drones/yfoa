/*
 * @Name:
 * @Author:       yong
 * @Date: 2022-10-19
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-12-18 17:41:40
 * @Description:
 * @Subscriber:
 * @Publisher:
 */
#include "MissionXYZ.h"

MissionXYZ::MissionXYZ() {}

MissionXYZ::~MissionXYZ() { yf_mission_file_.close(); }

void MissionXYZ::init(ros::NodeHandle node)
{
    node.param("mission/waypoint_num", wps_num_, -1);
    for (int i = 0; i < wps_num_; i++)
    {
        MAVState point;
        node.param("mission/waypoint" + std::to_string(i) + "_x", point.pos[0], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_y", point.pos[1], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_z", point.pos[2], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_vm", point.max_vel, -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_am", point.max_acc, -1.0);
        wps_.push_back(point);
    }
    std::cout << "[mission]  the waypoint number: " << wps_.size() << std::endl;

    node.param("mission/sendOneByOne", sendOneByOne_, true);
    node.param("mission/wps_threshold", wps_thr_, 2.0);
    node.param("mission/control_mode", control_mode_, 2);
    node.param<std::string>("mission/handle_wpts_xy", handle_wpts_xy_, "UseOffboardPoint");
    node.param<std::string>("mission/handle_wpts_z", handle_wpts_z_, "UseOffboardHeight");

    mission_fsm_timer_ = node.createTimer(ros::Duration(0.10), &MissionXYZ::missionCallback, this);

    state_sub_ = node.subscribe("/mavros/state", 10, &MissionXYZ::stateCallback, this);
    odom_sub_ = node.subscribe("/odom", 10, &MissionXYZ::odomCallback, this);
    rviz_sub_ = node.subscribe("/move_base_simple/goal", 10, &MissionXYZ::rvizCallback, this);
    bspline_sub_ = node.subscribe("/planner/bspline", 1, &MissionXYZ::bsplineCallback, this);
    planerflag_sub_ = node.subscribe("/planner/flag", 1, &MissionXYZ::planerFlagCallback, this);

    // wps_pub_ = node.advertise<yf_manager::WayPoints>("/mission/waypoints", 10);
    setpoint_raw_local_pub_ = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    poscmds_vis_pub_ = node.advertise<sensor_msgs::PointCloud2>("/mission/poscmds_vis", 10);
    posactual_vis_pub_ = node.advertise<sensor_msgs::PointCloud2>("/mission/posactual_vis", 10);

    has_odom_ = false;
    sendflag_ = true;
    k_ = 0;

    time_forward_ = 1.0;
    receive_traj_ = false;

    changeMissionState(mission_fsm_state_, MISSION_STATE::READY);

    pos_cmds_.empty();
    pos_actual_.empty();

    std::string filename;
    node.param<std::string>("mission/yf_mission_log_filename", filename, "~/yf_mission.txt");
    yf_mission_file_.open(filename);
    if (!yf_mission_file_)
    {
        std::cerr << "无法打开文件" << std::endl;
        exit(0);
    }
}

void MissionXYZ::missionCallback(const ros::TimerEvent &e)
{

    if (!has_odom_ && !has_depth_) // check inputs
    {
        // reset();
        return;
    }

    if (uav_sysstate_.armed == false) // check armming state
    {
        changeMissionState(mission_fsm_state_, MISSION_STATE::READY);
        // reset();
        return;
    }

    if (uav_sysstate_.mode != "OFFBOARD") // check flight
    {
        changeMissionState(mission_fsm_state_, MISSION_STATE::MANUALFLIGHT);
        // reset();
        return;
    }

    switch (mission_fsm_state_)
    {
    case MISSION_STATE::READY:
    {
        if (uav_sysstate_.armed == true && last_uav_sysstate_.armed == false)
        {
            // setHome(odom_, home_);
            home_ = current_state_;
            changeMissionState(mission_fsm_state_, MISSION_STATE::MANUALFLIGHT);
        }
        break;
    }

    case MISSION_STATE::MANUALFLIGHT:
    {
        if (uav_sysstate_.mode == "OFFBOARD" && last_uav_sysstate_.mode != "OFFBOARD")
        {
            // std::cout << "[mission]  the OFFBOARD position(x,y,z,yaw): " << pos_sp_.transpose() << ", " << yaw_sp_ * 53.7 << std::endl;

            for (int i = 0; i < wps_.size(); i++)
            {
                if (handle_wpts_xy_ == "UseArmmingPoint")
                {
                    wps_[i].pos[0] += home_.pos[0];
                    wps_[i].pos[1] += home_.pos[1];
                }
                else if (handle_wpts_xy_ == "UseOffboardPoint")
                {
                    wps_[i].pos[0] += odom_.pose.pose.position.x;
                    wps_[i].pos[1] += odom_.pose.pose.position.y;
                }

                if (handle_wpts_z_ == "UseSetHeight")
                    wps_[i].pos[2] += home_.pos[2];
                else if (handle_wpts_z_ == "UseOffboardHeight") // 当前高度作为目标点高度
                    wps_[i].pos[2] = odom_.pose.pose.position.z;
            }

            changeMissionState(mission_fsm_state_, MISSION_STATE::AUTOFLIGHT);
        }

        publishCmd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::VEL);

        break;
    }

    case MISSION_STATE::AUTOFLIGHT:
    {
        if (!receive_traj_)
        {
            mission_auto_fsm_state_ = MISSION_AUTO_STATE::GEN_NEW_TRAJ;
        }
        else
            mission_auto_fsm_state_ = MISSION_AUTO_STATE::EXEC_TRAJ;

        if (collision_)
            mission_auto_fsm_state_ = MISSION_AUTO_STATE::REPLAN_TRAJ;

        Eigen::Vector3d pos_sp, vel_sp, acc_sp;
        double yaw_sp;
        ros::Time time_now;
        static ros::Time time_last = ros::Time::now();
        switch (mission_auto_fsm_state_)
        {
        case MISSION_AUTO_STATE::GEN_NEW_TRAJ:
        {
            // 判断有没有达到途径点
            trajectory_.end_mavstate = choseTarget();

            trajectory_.start_time = current_state_.time;
            publishSE(current_state_, trajectory_.end_mavstate);
            publishCmd(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::VEL);
        }
        case MISSION_AUTO_STATE::REPLAN_TRAJ:
        {
            trajectory_.end_mavstate = choseTarget();

            time_now = ros::Time::now();
            double t_cur = (time_now - trajectory_.start_time).toSec();
            trajectory_.start_mavstate.pos = trajectory_.position_traj.evaluateDeBoorT(t_cur);
            trajectory_.start_mavstate.vel = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
            trajectory_.start_mavstate.acc = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
            trajectory_.start_time = time_now;
            publishSE(current_state_, trajectory_.end_mavstate);

            pos_sp = trajectory_.position_traj.evaluateDeBoorT(t_cur);
            vel_sp = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
            acc_sp = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
            yaw_sp = calculate_yaw(t_cur, pos_sp_, time_now, time_last).first;
            publishCmd(pos_sp, vel_sp, acc_sp, yaw_sp, control_mode_);
        }
        case MISSION_AUTO_STATE::EXEC_TRAJ:
        {
            time_now = ros::Time::now();

            // 发送指令
            double t_cur = (time_now - trajectory_.start_time).toSec();
            t_cur = std::min(trajectory_.duration, t_cur);
            if (t_cur > trajectory_.duration - 1e-2) // 轨迹结束
            {
                pos_sp = trajectory_.position_traj.evaluateDeBoorT(trajectory_.duration);
                publishCmd(pos_sp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0, ControlMode::POS);
                // changeFSMExecState(FsmState::WAIT_TARGET, "FSM");
                // return;
            }
            else
            {
                pos_sp = trajectory_.position_traj.evaluateDeBoorT(t_cur);
                vel_sp = trajectory_.velocity_traj.evaluateDeBoorT(t_cur);
                acc_sp = trajectory_.acceleration_traj.evaluateDeBoorT(t_cur);
                yaw_sp = calculate_yaw(t_cur, pos_sp_, time_now, time_last).first;
                publishCmd(pos_sp, vel_sp, acc_sp, yaw_sp, control_mode_);
            }

            // 判断是否需要重规划
            if ((trajectory_.end_mavstate.pos - pos_sp).norm() < no_replan_thresh_) // 到达终点附近，不再规划
            {
                // cout << "near end" << endl;
            }
            else if ((trajectory_.start_mavstate.pos - pos_sp).norm() > replan_thresh_) // 走过一段距离，需要重新规划
            {
                mission_auto_fsm_state_ = MISSION_AUTO_STATE::REPLAN_TRAJ;
            }

            break;
        }
        }
        time_last = time_now;
    }
    }
    last_uav_sysstate_ = uav_sysstate_;
}

// void MissionXYZ::setHome(MAVState odom, MAVState &home)
// {
//     home.pos[0] = odom_.pose.pose.position.x;
//     home.pos[1] = odom_.pose.pose.position.y;
//     home.pos[2] = odom_.pose.pose.position.z;

//     home.yaw = quaternion_to_yaw(odom_.pose.pose.orientation);
// }

void MissionXYZ::changeMissionState(int &mode, int next)
{
    mode = next;
    // std::cout << "[mission] mode " << next << std::endl;
    // static string state_str[7] = {"IDLE", "READY", "TAKEOFF", "MOVE", "LAND", "FAULT"};
    // std::cout << "\033[34m" << ros::Time::now() << "[mission] change mode to " << state_str[mode] << "\033[0m" << std::endl;
}

void MissionXYZ::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    uav_sysstate_ = *msg;

    if (uav_sysstate_.mode == "OFFBOARD" || uav_sysstate_.mode == "GUIDED" || uav_sysstate_.mode == "CMODE(4)")
        uav_sysstate_.mode = "OFFBOARD";
}

void MissionXYZ::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    current_state_.time = ros::Time::now();

    current_state_.pos(0) = msg->pose.pose.position.x;
    current_state_.pos(1) = msg->pose.pose.position.y;
    current_state_.pos(2) = msg->pose.pose.position.z;

    current_state_.vel(0) = msg->twist.twist.linear.x;
    current_state_.vel(1) = msg->twist.twist.linear.y;
    current_state_.vel(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    current_state_.quat.w() = msg->pose.pose.orientation.w;
    current_state_.quat.x() = msg->pose.pose.orientation.x;
    current_state_.quat.y() = msg->pose.pose.orientation.y;
    current_state_.quat.z() = msg->pose.pose.orientation.z;

    has_odom_ = true;
}

void MissionXYZ::bsplineCallback(yf_manager::BsplineConstPtr msg)
{
    // parse pos traj

    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
        knots(i) = msg->knots[i];
    }

    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    //   yaw_pts(i, 0) = msg->yaw_pts[i];
    // }

    // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());

    traj_duration_ = traj_[0].getTimeSum();

    receive_traj_ = true;
}

void MissionXYZ::planerFlagCallback(const std_msgs::Int16 &msg)
{
    if (msg.data == 2) // FsmState::GEN_NEW_TRAJ
    {
    }

    if (msg.data == 3) // FsmState::REPLAN_TRAJ
    {
        // const double time_out = 0.001;
        // ros::Time time_now = ros::Time::now();
        // double t_stop = (time_now - start_time_).toSec() + time_out;
        traj_duration_ = std::min((ros::Time::now() - start_time_).toSec() + 0.1, traj_duration_);
    }
}

void MissionXYZ::rvizCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (mission_fsm_state_ < MISSION_STATE::MOVE)
        return;
    yf_manager::WayPoints way_points;
    way_points.header.stamp = ros::Time::now();
    way_points.size = 1;

    ROS_WARN("[mission] MOVE, send rviz goal");

    way_points.pos_x.push_back(msg->pose.position.x);
    way_points.pos_y.push_back(msg->pose.position.y);
    way_points.pos_z.push_back(odom_.pose.pose.position.z);
    way_points.max_vel.push_back(wps_[0].max_vel);
    way_points.max_acc.push_back(wps_[0].max_acc);

    wps_pub_.publish(way_points);
}

std::pair<double, double> MissionXYZ::calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    Eigen::Vector3d dir = t_cur + time_forward_ <= trajectory_.duration ? trajectory_.position_traj.evaluateDeBoorT(t_cur + time_forward_) - pos : trajectory_.position_traj.evaluateDeBoorT(trajectory_.duration) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

void MissionXYZ::publishCmd(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, int cmode)
{
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.header.stamp = ros::Time::now();

    // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    // Bit 10 should set to 0, means is not force sp
    if (cmode == ControlMode::POS)
        pos_setpoint.type_mask = 0b110111111000;
    else if (cmode == ControlMode::POSYAW)
        pos_setpoint.type_mask = 0b100111111000;
    else if (cmode == ControlMode::POSVELYAW)
        pos_setpoint.type_mask = 0b100111000000;
    else if (cmode == ControlMode::POSVELACCYAW)
        pos_setpoint.type_mask = 0b100000000000;
    else if (cmode == ControlMode::VEL)
        pos_setpoint.type_mask = 0b110000000111;

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.velocity.x = vel_sp[0];
    pos_setpoint.velocity.y = vel_sp[1];
    pos_setpoint.velocity.z = vel_sp[2];
    pos_setpoint.acceleration_or_force.x = acc_sp[0];
    pos_setpoint.acceleration_or_force.y = acc_sp[1];
    pos_setpoint.acceleration_or_force.z = acc_sp[2];

    pos_setpoint.yaw = yaw_sp;

    setpoint_raw_local_pub_.publish(pos_setpoint);
}

double MissionXYZ::quaternion_to_yaw(geometry_msgs::Quaternion &q)
{
    // 四元数元素：w, x, y, z
    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;

    // 计算偏航角（yaw）
    double yaw = atan2(2.0 * (z * w + x * y), 1.0 - 2.0 * (y * y + z * z));
    return yaw;
}

void MissionXYZ::publishPoints(std::vector<Eigen::Vector3d> points, ros::Publisher pub)
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
