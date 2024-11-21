#ifndef _HYBIRDASTAR_H
#define _HYBIRDASTAR_H

#include <boost/functional/hash.hpp>
#include <iostream>
#include <queue>
#include <string>
#include <map>
#include <unordered_map>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Eigen>

#include "InESDFMap.hpp"

using namespace std;
using namespace Eigen;

// #define inf 1 >> 30

enum NodeState
{
  IN_CLOSE_SET,
  IN_OPEN_SET,
  NOT_EXPAND
};

class PathNode
{
public:
  /* -------------------- */
  Eigen::Vector3i index; // 用于比较两个节点是否位于同一个栅格中
  Eigen::Matrix<double, 6, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time; // dynamic
  int time_idx;
  PathNode *cameFrom;
  NodeState node_state;

  /* -------------------- */
  PathNode()
  {
    cameFrom = NULL;
    node_state = NodeState::NOT_EXPAND;
  }
  ~PathNode() {};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode *PathNodePtr;

class NodeComparator
{
public:
  bool operator()(PathNodePtr node1, PathNodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable
{
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodePtr, matrix_hash<Eigen::Vector4i>>
      data_4d_;

public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector3i idx, PathNodePtr node)
  {
    data_3d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodePtr node)
  {
    data_4d_.insert(std::make_pair(
        Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector3i idx)
  {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector3i idx, int time_idx)
  {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear()
  {
    data_3d_.clear();
    data_4d_.clear();
  }

  int size()
  {
    return data_3d_.size();
  }
};

/**
 * @brief 混合A*路径搜索
 * @ 结果是位置速度连续，加速度作为控制量不连续
 *
 */
class HybirdAstar
{
public:
  HybirdAstar() {};
  ~HybirdAstar();

  enum
  {
    REACH_HORIZON = 1,
    REACH_END = 2,
    NO_PATH = 3,
    NEAR_END = 4
  };

  /* main API */
  void init(std::string filename, const InESDFMap::Ptr map_ptr, bool verbose = false);

  /**
   * @brief 混合A*搜索
   *
   * @param init TRUE表示使用设定的初始加速度，FALSE表示使用离散的最大加速度采样。当初始加速度为0时，init需要为false，否则跑不出第一个栅格
   * @param horizon 表示搜索轨迹的最大长度
   * @param dynamic  表示搜索的轨迹是否有时间戳
   * @param time_start 表示搜索的轨迹时间戳的起始时间
   */
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel,
             bool init, double horizon, bool dynamic = false, double time_start = -1.0);

  void setPhysicLimits(double max_vel, double max_acc)
  {
    max_vel_ = max_vel;
    max_acc_ = max_acc;
  };

  /**
   * @brief 获取混合A*搜索结果的轨迹
   *
   * @param delta_t 采样的时间间隔
   */
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);

  /** @brief 获取混合A*搜索结果的动作序列     */
  std::vector<PathNodePtr> getPathNodes();

  /** @brief 获取混合A*搜索遍历过的节点     */
  std::vector<PathNodePtr> getVisitedNodes();
  std::vector<Eigen::Vector3d> getVisitedPath(double delta_t);

  std::vector<Eigen::Vector3d> getAllMotions(double delta_t);
  std::vector<std::pair<Eigen::Matrix<double, 6, 1>, Eigen::Vector4d>> all_motions_;

  // void parameterTraj(double start_time, double delta_t);
  // void saveTrajToTxt(std::string filename);
  // QuinticSpline trajectory_;

  typedef std::shared_ptr<HybirdAstar> Ptr;

  // #ifdef MY_DEBUG
  //     ros::Publisher paths_pub_;
  // #endif

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  InESDFMap::Ptr map_ptr_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;

  /* search */
  double max_tau_, init_max_tau_; // 状态传播的最大时间间隔
  double max_vel_, max_acc_;
  double min_dist_;
  double w_time_, lambda_heu_;

  int check_num_; // 碰撞检查的数量

  double tie_breaker_;
  // bool optimistic_;

  double time_origin_;

  /* ---------- main data structure ---------- */
  int allocate_num_;
  std::vector<PathNodePtr> path_node_pool_; //  预先分配的节点

  int use_node_num_;                    // = path_nodes_.size() + open_set_.size()
  std::vector<PathNodePtr> path_nodes_; // 记录结果的节点
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;

  NodeHashTable expanded_nodes_; // 记录所有遍历过的节点，为了查找节点时是否已经被遍历

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;

  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;

  bool verbose_;
  int iter_num_;

  /* helper */
  Eigen::Vector3i PosToIndex(Eigen::Vector3d pt);
  int TimeToIndex(double time);
  void retrievePath(PathNodePtr end_node, std::vector<PathNodePtr> &path_nodes);

  /**
   * @brief 使用多项式 (a*t^3 + b*t^2 + v0*t + p0) 计算一段直线轨迹
   * @param time_to_goal 表示该段轨迹的期望用时。
   * @param coef_shot 表示该段轨迹的多项式系数。
   * @param t_shot 表示该段轨迹的历时。
   */
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal, Eigen::MatrixXd &coef_shot, double &t_shot);

  /** @brief 基于庞特利亚金最小值原理的启发式函数
   * @cost J = \int u^2 dt + \rho T = -c1/(3*T^3) - c2/(2*T^2) - c3/T + w_time_*T;
   */
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quarticRoots(double a, double b, double c, double d, double e);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time, double max_vel);

  /**
   * @brief state propagation
   * @param um 控制量（加速度）
   * @param tau 持续时间
   * @param state0 当前的状态（位置和速度）
   * @param state1 下一时刻的状态（位置和速度）
   */
  void stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau);

  /*** 清除上次搜索过程产生的中间数据 */
  void clearLastSearchData();

  bool isOccupied(Eigen::Vector3d pos, double thr = -1.0);
};

#endif