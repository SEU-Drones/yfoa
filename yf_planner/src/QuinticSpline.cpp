#include "QuinticSpline.h"

Eigen::MatrixXd QuinticSpline::PCBlock(double t)
{
    Eigen::MatrixXd coeff;
    coeff.resize(1, 6);
    for (int i = 0; i < 6; i++)
    {
        coeff(0, i) = std::pow(t, i);
    }

    return coeff;
}

Eigen::MatrixXd QuinticSpline::VCBlock(double t)
{
    Eigen::MatrixXd coeff;
    coeff.resize(1, 6);
    for (int i = 0; i < 6; i++)
    {
        coeff(0, i) = i * std::pow(t, i - 1); // 要求t!=0
    }

    return coeff;
}

Eigen::MatrixXd QuinticSpline::ACBlock(double t)
{
    Eigen::MatrixXd coeff;
    coeff.resize(1, 6);

    for (int i = 0; i < 6; i++)
    {
        coeff(0, i) = i * (i - 1) * std::pow(t, i - 2);
    }

    return coeff;
}

Eigen::MatrixXd QuinticSpline::JCBlock(double t)
{
    Eigen::MatrixXd coeff;
    coeff.resize(1, 6);

    for (int i = 0; i < 6; i++)
    {
        coeff(0, i) = i * (i - 1) * (i - 2) * std::pow(t, i - 3);
    }
    return coeff;
}

Eigen::MatrixXd QuinticSpline::SCBlock(double t)
{
    Eigen::MatrixXd coeff;
    coeff.resize(1, 6);
    for (int i = 0; i < 6; i++)
    {
        coeff(0, i) = i * (i - 1) * (i - 2) * (i - 3) * std::pow(t, i - 4);
    }
    return coeff;
}

int QuinticSpline::locateIndex(double t)
{
    if (t < time_pts_[0])
    {
        std::cerr << "Error input time @ " << t << std::endl;
        return NAN;
    }

    int index = 0;
    double temp = time_pts_[1];
    while (t > temp)
    {
        index++;
        temp = time_pts_[index + 1];
    }

    return index;
}

bool QuinticSpline::QuinticSplineInterpolation(int dimension, Eigen::MatrixXd wps, std::vector<double> time_pts, Eigen::MatrixXd bounders)
{
    if (wps.rows() < 2)
    {
        std::cerr << "[QuinticSpline] The number of waypoints is too small!" << std::endl;
        return false;
    }

    if (bounders.rows() < 4)
    {
        std::cerr << "[QuinticSpline] The number of bounders is too small! At least 4" << std::endl;
        return false;
    }

    // std::cout << "wps: " << std::endl;
    // std::cout << wps << std::endl;

    dim_ = dimension;
    time_pts_ = time_pts;
    coeffs_.resize(dim_);

    int M = time_pts_.size(); // M个途径点
    int N = 5;                // N次多项式

    // 对每个维度单独求解
    for (int dim = 0; dim < dim_; dim++)
    {
        // 初始化每个维度的系数参数
        coeffs_[dim].resize(M - 1, N + 1);
        coeffs_[dim].setZero();
        // std::cout << "Rows: " << coeffs_[dim].rows() << ", Cols: " << coeffs_[dim].cols() << std::endl;

        // 构造方程 Ax=b
        Eigen::VectorXd x = Eigen::VectorXd::Zero((N + 1) * (M - 1));
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero((N + 1) * (M - 2) + 2 + bounders.rows(), (N + 1) * (M - 1));
        Eigen::VectorXd b = Eigen::VectorXd::Zero((N + 1) * (M - 2) + 2 + bounders.rows());

        // M-2个中间点, 6 * (M - 2)个连续条件等式；
        for (int i = 0; i < M - 2; i++)
        {
            b[(N + 1) * i + 5] = 0; // snap
            A.block((N + 1) * i + 5, i * (N + 1), 1, N + 1) = SCBlock(time_pts[i + 1]);
            A.block((N + 1) * i + 5, (i + 1) * (N + 1), 1, N + 1) = -SCBlock(time_pts[i + 1]);

            b[(N + 1) * i + 4] = 0; // jerk
            A.block((N + 1) * i + 4, i * (N + 1), 1, N + 1) = JCBlock(time_pts[i + 1]);
            A.block((N + 1) * i + 4, (i + 1) * (N + 1), 1, N + 1) = -JCBlock(time_pts[i + 1]);

            b[(N + 1) * i + 3] = 0; // acc连续
            A.block((N + 1) * i + 3, i * (N + 1), 1, N + 1) = ACBlock(time_pts[i + 1]);
            A.block((N + 1) * i + 3, (i + 1) * (N + 1), 1, N + 1) = -ACBlock(time_pts[i + 1]);

            b[(N + 1) * i + 2] = 0; // vel连续
            A.block((N + 1) * i + 2, i * (N + 1), 1, N + 1) = VCBlock(time_pts[i + 1]);
            A.block((N + 1) * i + 2, (i + 1) * (N + 1), 1, N + 1) = -VCBlock(time_pts[i + 1]);

            b[(N + 1) * i + 1] = wps(i + 1, dim); // wps右端点
            A.block((N + 1) * i, i * (N + 1), 1, N + 1) = PCBlock(time_pts[i + 1]);

            b[(N + 1) * i] = wps(i + 1, dim); // wps左端点
            A.block((N + 1) * i + 1, (i + 1) * (N + 1), 1, N + 1) = PCBlock(time_pts[i + 1]);
        }

        // 起点和终点的位置约束
        A.block((N + 1) * (M - 2), 0, 1, N + 1) = PCBlock(time_pts[0]); // 起点位置
        b[(N + 1) * (M - 2)] = wps(0, dim);

        A.block((N + 1) * (M - 2) + 1, 0, 1, N + 1) = VCBlock(time_pts[0]); // 起点速度
        A((N + 1) * (M - 2) + 1, 0) = 0;
        b[(N + 1) * (M - 2) + 1] = bounders(0, dim);

        A.block((N + 1) * (M - 2) + 2, 0, 1, N + 1) = ACBlock(time_pts[0]); // 起点加速度
        A((N + 1) * (M - 2) + 2, 0) = 0;
        A((N + 1) * (M - 2) + 2, 1) = 0;
        b[(N + 1) * (M - 2) + 2] = bounders(1, dim);

        A.block((N + 1) * (M - 2) + 3, (N + 1) * (M - 2), 1, N + 1) = PCBlock(time_pts[time_pts.size() - 1]); // 终点位置
        b[(N + 1) * (M - 2) + 3] = wps(wps.rows() - 1, dim);

        A.block((N + 1) * (M - 2) + 4, (N + 1) * (M - 2), 1, N + 1) = VCBlock(time_pts[time_pts.size() - 1]); // 终点速度
        b[(N + 1) * (M - 2) + 4] = bounders(2, dim);

        A.block((N + 1) * (M - 2) + 5, (N + 1) * (M - 2), 1, N + 1) = ACBlock(time_pts[time_pts.size() - 1]); // 终点加速度
        b[(N + 1) * (M - 2) + 5] = bounders(3, dim);

        // std::cout << "A: " << std::endl;
        // std::cout << A << std::endl;
        // std::cout << "b: " << std::endl;
        // std::cout << b << std::endl;

        // 解方程Ax=b
        x = A.colPivHouseholderQr().solve(b);

        for (int i = 0; i < M - 1; i++)
        {
            for (int j = 0; j < N + 1; j++)
            {
                if (std::isnan(x[i * (N + 1) + j]))
                {
                    std::cerr << "[QuinticSpline] The coeffs has a nan!" << std::endl;
                    return false;
                }
                coeffs_[dim](i, j) = x[i * (N + 1) + j];
            }
        }
        // std::cout << "coeffs: " << std::endl;
        // std::cout << coeffs_[dim] << std::endl;
    }

    return true;
}

bool QuinticSpline::sampleYaw(double t, double &yaw, double t_forward)
{
    Eigen::VectorXd pos1, pos2;

    if (t + t_forward < time_pts_[time_pts_.size() - 1])
    {
        samplePos(t, pos1);
        samplePos(t + t_forward, pos2);

        yaw = std::atan2(pos2(1) - pos1(1), pos2(0) - pos1(0)); //(-pi,pi)
    }
    else
    {
        Eigen::VectorXd vel;
        sampleVel(t, vel);

        yaw = std::atan2(vel(1), vel(0)); //(-pi,pi)
    }

    return true;
}

bool QuinticSpline::samplePos(double t, Eigen::VectorXd &pos)
{
    if (t < time_pts_[0] || t > time_pts_[time_pts_.size() - 1])
        return false;

    int index = locateIndex(t);

    pos.resize(dim_);
    for (int dim = 0; dim < dim_; dim++)
    {
        pos[dim] = 0;
        for (int i = 0; i < 6; i++)
            pos[dim] += coeffs_[dim](index, i) * std::pow(t, i);
    }

    return true;
}

bool QuinticSpline::sampleVel(double t, Eigen::VectorXd &vel)
{
    if (t < time_pts_[0] || t > time_pts_[time_pts_.size() - 1])
        return false;

    if (t == 0.0)
        t = t + 1e-3; // std::pow(0,负数)是未定义的

    int index = locateIndex(t);

    vel.resize(dim_);
    for (int dim = 0; dim < dim_; dim++)
    {
        vel[dim] = 0;
        for (int i = 0; i < 6; i++)
            vel[dim] += i * coeffs_[dim](index, i) * std::pow(t, i - 1);
    }

    return true;
}

bool QuinticSpline::sampleAcc(double t, Eigen::VectorXd &acc)
{
    if (t < time_pts_[0] || t > time_pts_[time_pts_.size() - 1])
        return false;

    if (t == 0.0)
        t = t + 1e-3; // std::pow(0,负数)是未定义的

    int index = locateIndex(t);

    acc.resize(dim_);
    for (int dim = 0; dim < dim_; dim++)
    {
        acc[dim] = 0;
        for (int i = 0; i < 6; i++)
            acc[dim] += i * (i - 1) * coeffs_[dim](index, i) * std::pow(t, i - 2);
    }

    return true;
}

bool QuinticSpline::samplePos(double t, Eigen::Vector3d &pos)
{
    if (t < time_pts_[0] || t > time_pts_[time_pts_.size() - 1])
        return false;

    int index = locateIndex(t);

    for (int dim = 0; dim < dim_; dim++)
    {
        pos[dim] = 0;
        for (int i = 0; i < 6; i++)
            pos[dim] += coeffs_[dim](index, i) * std::pow(t, i);
    }

    return true;
}

bool QuinticSpline::sampleVel(double t, Eigen::Vector3d &vel)
{
    if (t < time_pts_[0] || t > time_pts_[time_pts_.size() - 1])
        return false;

    if (t == 0.0)
        t = t + 1e-3; // std::pow(0,负数)是未定义的

    int index = locateIndex(t);

    for (int dim = 0; dim < dim_; dim++)
    {
        vel[dim] = 0;
        for (int i = 0; i < 6; i++)
            vel[dim] += i * coeffs_[dim](index, i) * std::pow(t, i - 1);
    }

    return true;
}

bool QuinticSpline::sampleAcc(double t, Eigen::Vector3d &acc)
{
    if (t < time_pts_[0] || t > time_pts_[time_pts_.size() - 1])
        return false;

    if (t == 0.0)
        t = t + 1e-3; // std::pow(0,负数)是未定义的

    int index = locateIndex(t);

    for (int dim = 0; dim < dim_; dim++)
    {
        acc[dim] = 0;
        for (int i = 0; i < 6; i++)
            acc[dim] += i * (i - 1) * coeffs_[dim](index, i) * std::pow(t, i - 2);
    }

    return true;
}
