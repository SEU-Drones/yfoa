#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Eigen>

class QuinticSpline
{
private:
    int dim_; // 轨迹点的维度
    std::vector<double> time_pts_;
    std::vector<Eigen::MatrixXd> coeffs_; // 存储每段多项式的系数，假设每段都是3次，维度是3x[Nx6]

    // help functions
    Eigen::MatrixXd PCBlock(double t);
    Eigen::MatrixXd VCBlock(double t);
    Eigen::MatrixXd ACBlock(double t);
    Eigen::MatrixXd JCBlock(double t);
    Eigen::MatrixXd SCBlock(double t);

    int locateIndex(double t);

public:
    // 构造函数
    QuinticSpline() {};

    QuinticSpline(int dim) : dim_(dim) {};

    QuinticSpline(int dim, std::vector<double> time_pts, std::vector<Eigen::MatrixXd> coeffs)
    {
        dim_ = dim;
        time_pts_ = time_pts;
        coeffs_ = coeffs;
    };

    /*
    @brief : 基本插值，给定途径点的位置和2个边界条件
    @param dimension : 维度
    @param wps : 输入的经过点，每一行表示一个点
    @param time_pts : 到达每个位置的时间点，一般从0开始
    @param bounders : 边界条件，起点和终点的速度和加速度
    */
    bool QuinticSplineInterpolation(int dimension, Eigen::MatrixXd wps, std::vector<double> time_pts, Eigen::MatrixXd bounders);

    /*
    @brief : 给定途径点的位置、速度，插值
    to do ...
    */
    bool QuinticSplineInterpolation(int dimension, Eigen::MatrixXd wps, std::vector<double> time_pts);

    // 采样函数
    bool sampleYaw(double t, double &yaw, double t_forward = 0.1);

    bool samplePos(double t, Eigen::VectorXd &pos);

    bool sampleVel(double t, Eigen::VectorXd &vel);

    bool sampleAcc(double t, Eigen::VectorXd &acc);

    bool samplePos(double t, Eigen::Vector3d &pos);

    bool sampleVel(double t, Eigen::Vector3d &vel);

    bool sampleAcc(double t, Eigen::Vector3d &acc);

    int getDim() { return dim_; };

    std::vector<Eigen::MatrixXd> getCoeffs() { return coeffs_; };
    std::vector<double> getTimePts() { return time_pts_; };
};
