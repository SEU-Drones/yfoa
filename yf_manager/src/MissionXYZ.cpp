/*
 * @Name:
 * @Author:       yong
 * @Date: 2022-10-19
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2024-11-25 17:16:27
 * @Description:
 * @Subscriber:
 * @Publisher:
 */
#include "MissionXYZ.h"

MissionXYZ::MissionXYZ() {}

MissionXYZ::~MissionXYZ() {}

void MissionXYZ::init(ros::NodeHandle node)
{
    node.param("mission/waypoint_num", wps_num_, -1);
    for (int i = 0; i < wps_num_; i++)
    {
        Point point;
        node.param("mission/waypoint" + std::to_string(i) + "_x", point.pos[0], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_y", point.pos[1], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_z", point.pos[2], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_vx", point.vel[0], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_vy", point.vel[1], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_vz", point.vel[2], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_ax", point.acc[0], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_ay", point.acc[1], -1.0);
        node.param("mission/waypoint" + std::to_string(i) + "_az", point.acc[2], -1.0);
        wps_.push_back(point);
    }
    std::cout << "[mission]  the waypoint number: " << wps_.size() << std::endl;

    node.param("mission/sendOneByOne", sendOneByOne_, true);
    node.param("mission/wps_threshold", wps_thr_, 2.0);
    node.param("mission/control_mode", control_mode_, 2);
    node.param<std::string>("mission/handle_wpts_xy", handle_wpts_xy_, "UseOffboardPoint");
    node.param<std::string>("mission/handle_wpts_z", handle_wpts_z_, "UseOffboardHeight");

    

    mission_fsm_timer_ = node.createTimer(ros::Duration(0.10), &MissionXYZ::missionCallback, this);
    cmd_timer_ = node.createTimer(ros::Duration(0.05), &MissionXYZ::cmdCallback, this);

    state_sub_ = node.subscribe("/mavros/state", 10, &MissionXYZ::stateCallback, this);
    odom_sub_ = node.subscribe("/odom", 10, &MissionXYZ::localOdomCallback, this);
    rviz_sub_ = node.subscribe("/move_base_simple/goal", 10, &MissionXYZ::rvizCallback, this);
    bspline_sub_ = node.subscribe("/planner/bspline", 1, &MissionXYZ::bsplineCallback, this);

    wps_pub_ = node.advertise<yf_manager::WayPoints>("/mission/waypoints", 10, this);
    setpoint_raw_local_pub_ = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    poscmds_vis_pub_ = node.advertise<sensor_msgs::PointCloud2>("/mission/poscmds_vis", 10);
    posactual_vis_pub_ = node.advertise<sensor_msgs::PointCloud2>("/mission/posactual_vis", 10);

    has_odom_ = false;
    sendflag_ = true;
    k_ = 0;
    traj_id_ = 0;

    time_forward_ = 1.0;
    receive_traj_ = false;

    changeMissionState(mission_fsm_state_, MISSION_STATE::IDLE);

    pos_cmds_.empty();
    pos_actual_.empty();
}

void MissionXYZ::missionCallback(const ros::TimerEvent &e)
{
    if (!has_odom_)
    {
        ROS_INFO("no odom!");
        return;
    }

    switch (mission_fsm_state_)
    {

    case MISSION_STATE::IDLE:
    {
        if (state_.armed == true)
            changeMissionState(mission_fsm_state_, MISSION_STATE::READY);
        break;
    }

    case MISSION_STATE::READY:
    {
        setHome(odom_, home_);
        // handleWaypoints(wps_, home_);
        ROS_WARN("[mission] Arimming!");
        std::cout << "[mission]  the home position(x,y,z,yaw): " << home_.pos.transpose() << ", " << home_.yaw * 53.7 << std::endl;

        changeMissionState(mission_fsm_state_, MISSION_STATE::TAKEOFF);
        break;
    }

    case MISSION_STATE::TAKEOFF:
    {
        pos_sp_(0) = odom_.pose.pose.position.x;
        pos_sp_(1) = odom_.pose.pose.position.y;
        pos_sp_(2) = odom_.pose.pose.position.z;
        yaw_sp_ = quaternion_to_yaw(odom_.pose.pose.orientation);

        if (state_.mode == "OFFBOARD")
        {
            std::cout << "[mission]  the OFFBOARD position(x,y,z,yaw): " << pos_sp_.transpose() << ", " << yaw_sp_ * 53.7 << std::endl;
            // 当前高度作为目标点高度
            for (int i = 0; i < wps_.size(); i++){
                if(handle_wpts_xy_ == "UseArmmingPoint"){
                    wps_[i].pos[0] += home_.pos[0];
                    wps_[i].pos[1] += home_.pos[1];
                }
                else if(handle_wpts_xy_ == "UseOffboardPoint"){
                    wps_[i].pos[0] += odom_.pose.pose.position.z;
                    wps_[i].pos[1] += odom_.pose.pose.position.y;
                }

                if(handle_wpts_z_ == "UseSetHeight")
                    wps_[i].pos[2] += home_.pos[2];
                else if(handle_wpts_z_ == "UseOffboardHeight")
                    wps_[i].pos[2] = odom_.pose.pose.position.z;

            }

            changeMissionState(mission_fsm_state_, MISSION_STATE::MOVE);
        }

        if (state_.armed == false)
            changeMissionState(mission_fsm_state_, MISSION_STATE::IDLE);
        break;
    }

    case MISSION_STATE::MOVE:
    {
        if (sendOneByOne_)
        {
            if (sendflag_)
            {
                sendflag_ = false;
                sendWayPoints(wps_, k_);
            }

            // 途径点判断
            if (k_ < wps_.size() - 1)
            {
                if (sqrt(pow(odom_.pose.pose.position.x - wps_[k_].pos[0], 2) +
                         pow(odom_.pose.pose.position.y - wps_[k_].pos[1], 2) +
                         pow(odom_.pose.pose.position.z - wps_[k_].pos[2], 2)) < wps_thr_)
                {
                    k_++;
                    sendflag_ = true;
                }
            }
        }
        else
        {
            if (sendflag_)
            {
                sendflag_ = false;
                k_ = wps_.size();
                sendWayPoints(wps_);
            }
        }

        // 终点判断
        if (k_ == wps_.size() - 1)
        {
            if (sqrt(pow(odom_.pose.pose.position.x - wps_[k_ - 1].pos[0], 2) +
                     pow(odom_.pose.pose.position.y - wps_[k_ - 1].pos[1], 2) +
                     pow(odom_.pose.pose.position.z - wps_[k_ - 1].pos[2], 2)) < 1.0 &&
                sqrt(pow(odom_.twist.twist.linear.x, 2) +
                     pow(odom_.twist.twist.linear.y, 2) +
                     pow(odom_.twist.twist.linear.z, 2)) < 0.1)
            {
                changeMissionState(mission_fsm_state_, MISSION_STATE::LAND);
                sendflag_ = true;
                k_ = 0;
            }
        }
        break;
    }

    case MISSION_STATE::LAND:
    {
        if (state_.armed == false)
            changeMissionState(mission_fsm_state_, MISSION_STATE::IDLE);
        break;
    }
    }
}

void MissionXYZ::setHome(nav_msgs::Odometry odom, Point &home)
{
    home.pos[0] = odom_.pose.pose.position.x;
    home.pos[1] = odom_.pose.pose.position.y;
    home.pos[2] = odom_.pose.pose.position.z;

    home.yaw = quaternion_to_yaw(odom_.pose.pose.orientation);
}

void MissionXYZ::sendWayPoints(std::vector<Point> wayPoints, int k)
{
    yf_manager::WayPoints way_points;
    way_points.header.stamp = ros::Time::now();
    way_points.size = 1;

    ROS_WARN("[mission] MOVE, send goal");

    way_points.pos_x.push_back(wps_[k].pos[0]);
    way_points.pos_y.push_back(wps_[k].pos[1]);
    way_points.pos_z.push_back(wps_[k].pos[2]);
    way_points.max_vel.push_back(wps_[k].vel[0]);
    way_points.max_acc.push_back(wps_[k].acc[0]);

    std::cout << "[mission] send the " << k << "th goal: "
              << "   " << way_points.pos_x[0] << " " << way_points.pos_y[0] << " " << way_points.pos_z[0]
              << "   The max vel and acc are:  " << way_points.max_vel[0] << " " << way_points.max_acc[0] << std::endl;

    wps_pub_.publish(way_points);
}

void MissionXYZ::sendWayPoints(std::vector<Point> wayPoints)
{
    yf_manager::WayPoints way_points;
    way_points.header.stamp = ros::Time::now();
    way_points.size = wayPoints.size();

    ROS_WARN("[mission] MOVE, send goal");
    for (int k = 0; k < wayPoints.size(); k++)
    {
        way_points.pos_x.push_back(wps_[k].pos[0]);
        way_points.pos_y.push_back(wps_[k].pos[1]);
        way_points.pos_z.push_back(wps_[k].pos[2]);
        way_points.max_vel.push_back(wps_[k].vel[0]);
        way_points.max_acc.push_back(wps_[k].acc[0]);
        std::cout << "[mission] The " << k << "th goal: "
                  << "   " << way_points.pos_x[k] << " " << way_points.pos_y[k] << " " << way_points.pos_z[k]
                  << "   The max vel and acc are:  " << way_points.max_vel[k] << " " << way_points.max_acc[k] << std::endl;
    }

    wps_pub_.publish(way_points);
}

void MissionXYZ::changeMissionState(int &mode, int next)
{
    mode = next;
    std::cout << "[mission] mode " << next << std::endl;
}

void MissionXYZ::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    state_ = *msg;

    if (state_.mode == "OFFBOARD" || state_.mode == "GUIDED" || state_.mode == "CMODE(4)")
        state_.mode = "OFFBOARD";

    if (state_.armed == false)
        changeMissionState(mission_fsm_state_, MISSION_STATE::IDLE);
}

void MissionXYZ::localOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    has_odom_ = true;
    odom_ = *msg;
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
    way_points.max_vel.push_back(wps_[0].vel[0]);
    way_points.max_acc.push_back(wps_[0].acc[0]);

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

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
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

void MissionXYZ::sendCmd(Eigen::Vector3d pos_sp, Eigen::Vector3d vel_sp, Eigen::Vector3d acc_sp, double yaw_sp, int cmode)
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

void MissionXYZ::cmdCallback(const ros::TimerEvent &e)
{
    if (!receive_traj_)
    {
        // 进入offboard但是还没有生成路径时，发送以下期望点
        sendCmd(Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{0, 0, 0}, Eigen::Vector3d{0, 0, 0}, 0, ControlMode::VEL);
    }
    else
    {
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();

        // Eigen::Vector3d pos_f;
        std::pair<double, double> yaw_yawdot(0, 0);

        static ros::Time time_last = ros::Time::now();
        if (t_cur < traj_duration_ && t_cur >= 0.0)
        {
            pos_sp_ = traj_[0].evaluateDeBoorT(t_cur);
            vel_sp_ = traj_[1].evaluateDeBoorT(t_cur);
            acc_sp_ = traj_[2].evaluateDeBoorT(t_cur);

            /*** calculate yaw ***/
            yaw_yawdot = calculate_yaw(t_cur, pos_sp_, time_now, time_last);
            /*** calculate yaw ***/

            // double tf = std::min(traj_duration_, t_cur + 2.0);
            // pos_f = traj_[0].evaluateDeBoorT(tf);
        }
        else if (t_cur >= traj_duration_)
        {
            /* hover when finish traj_ */
            pos_sp_ = traj_[0].evaluateDeBoorT(traj_duration_);
            vel_sp_.setZero();
            acc_sp_.setZero();

            yaw_yawdot.first = last_yaw_;
            yaw_yawdot.second = 0;

            // pos_f = pos_sp_;
        }
        else
        {
            cout << "[Traj server]: invalid time." << endl;
        }
        time_last = time_now;

        yaw_sp_ = yaw_yawdot.first;

        sendCmd(pos_sp_, vel_sp_, acc_sp_, yaw_sp_, control_mode_);

        // std::cout << traj_id_ << "    " << ros::Time::now().toSec() << " " << t_cur << " " << pos_sp_.transpose() << "  " << vel_sp_.transpose() << "  " << acc_sp_.transpose() << std::endl;
        // std::cout << "[mission] " << yaw_sp_ << ", " << yaw_sp_ * 53.7 << std::endl;

        pos_cmds_.push_back(pos_sp_);
        publishPoints(pos_cmds_, poscmds_vis_pub_);
        if (pos_cmds_.size() > 10000)
            pos_cmds_.clear();
    }

    pos_actual_.push_back(Eigen::Vector3d{odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z});
    publishPoints(pos_actual_, posactual_vis_pub_);
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
