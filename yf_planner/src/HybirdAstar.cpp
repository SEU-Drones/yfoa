#include "HybirdAstar.h"

HybirdAstar::~HybirdAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

void HybirdAstar::init(std::string filename, const InESDFMap::Ptr map_ptr, bool verbose)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::cerr << "**ERROR CAN NOT OPEN YAML FILE**" << std::endl;
  }

  cv::FileNode yaml_node = fs["HybirdAstar"];
  max_tau_ = (double)(yaml_node["max_tau"]);
  init_max_tau_ = (double)(yaml_node["init_max_tau"]);
  max_vel_ = (double)(yaml_node["max_vel"]);
  max_acc_ = (double)(yaml_node["max_acc"]);
  min_dist_ = (double)(yaml_node["min_dist"]);
  w_time_ = (double)(yaml_node["w_time"]);
  resolution_ = (double)(yaml_node["resolution"]);
  time_resolution_ = (double)(yaml_node["time_resolution"]);
  lambda_heu_ = (double)(yaml_node["lambda_heu"]);
  allocate_num_ = (double)(yaml_node["allocate_num"]);
  check_num_ = (int)(yaml_node["check_num"]);
  no_search_dist_ = (double)(yaml_node["no_search_dist"]);

  cost_axis_weight_ = Eigen::Matrix3d::Identity();
  cost_axis_weight_(0, 0) = (double)(yaml_node["cost_axis_weight_x"]);
  cost_axis_weight_(1, 1) = (double)(yaml_node["cost_axis_weight_y"]);
  cost_axis_weight_(2, 2) = (double)(yaml_node["cost_axis_weight_z"]);

  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  vel_margin = (double)(yaml_node["vel_margin"]);
  max_vel_ += vel_margin;

  /* ---------- map params ---------- */
  inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  map_ptr_ = map_ptr;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new HybirdAstarPathNode;
  }

  // phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;

  verbose_ = verbose;

  std::cout << "[HybirdAstar INIT] max_tau: " << max_tau_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] init_max_tau: " << init_max_tau_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] max_vel: " << max_vel_ << " (m/s)" << std::endl;
  std::cout << "[HybirdAstar INIT] max_acc: " << max_acc_ << " (m/s^2)" << std::endl;
  std::cout << "[HybirdAstar INIT] min_dist: " << min_dist_ << " (m)" << std::endl;
  std::cout << "[HybirdAstar INIT] w_time: " << w_time_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] resolution: " << resolution_ << " (m)" << std::endl;
  std::cout << "[HybirdAstar INIT] time_resolution: " << time_resolution_ << " (s)" << std::endl;
  std::cout << "[HybirdAstar INIT] lambda_heu: " << lambda_heu_ << std::endl;
  std::cout << "[HybirdAstar INIT] allocate_num: " << allocate_num_ << std::endl;
  std::cout << "[HybirdAstar INIT] check_num: " << check_num_ << std::endl;
  std::cout << "[HybirdAstar INIT] no_search_dist: " << no_search_dist_ << std::endl;
}

void HybirdAstar::clearLastSearchData()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<HybirdAstarPathNodePtr, std::vector<HybirdAstarPathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    HybirdAstarPathNodePtr node = path_node_pool_[i];
    node->cameFrom = NULL;
    node->node_state = NodeState::NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  // has_path_ = false;
}

bool HybirdAstar::isOccupied(Eigen::Vector3d pos, double thr)
{
  if (thr < 0)
    return map_ptr_->isOccupied(pos);
  else
  {
    if (map_ptr_->getDist(pos) < thr)
      return true;
    else
      return false;
  }
}

int HybirdAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                        Eigen::Vector3d end_pt, Eigen::Vector3d end_v,
                        bool init, double horizon, bool dynamic, double time_start)
{
  clearLastSearchData();

  // handle start and end
  if (start_acc_.norm() < 0.1)
  {
    init = false;
    std::cout << "[hybird astar]: start acceleration is too small. And convert to discrete acceleration initialization! " << std::endl;
  }

  start_vel_ = start_v;
  start_acc_ = start_a;

  HybirdAstarPathNodePtr cur_node = path_node_pool_[0];
  cur_node->cameFrom = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = PosToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = PosToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal, max_vel_);
  cur_node->node_state = NodeState::IN_OPEN_SET;
  use_node_num_ += 1;
  open_set_.push(cur_node);

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = TimeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
  {
    expanded_nodes_.insert(cur_node->index, cur_node);
  }

  // HybirdAstarPathNodePtr neighbor = NULL;
  HybirdAstarPathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(no_search_dist_ / resolution_);

  while (!open_set_.empty())
  {
    cur_node = open_set_.top();
    open_set_.pop();

    // Terminate?
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node, path_nodes_);

      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal, max_vel_);
        is_shot_succ_ = computeShotTraj(cur_node->state, end_state, time_to_goal, coef_shot_, t_shot_);
        if (init_search)
        {
          std::cout << "[hybird astar] ERROR, Shot in first search loop!" << std::endl;
        }
      }
    }

    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "[hybird astar]: reach end" << std::endl;
        std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "[hybird astar]: reach horizon, over maximum trajectory length" << std::endl;
        std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "[hybird astar]: reach end" << std::endl;
        std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return REACH_END;
      }
      else if (cur_node->cameFrom != NULL)
      {
        std::cout << "[hybird astar]: near end" << std::endl;
        std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "[hybird astar]: no path" << std::endl;
        std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
        return NO_PATH;
      }
    }

    cur_node->node_state = NodeState::IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    std::vector<HybirdAstarPathNodePtr> tmp_expand_nodes;
    std::vector<Eigen::Vector3d> inputs;
    std::vector<double> durations;
    Eigen::Vector3d um;

    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
      {
        durations.push_back(tau);
      }

      init_search = false;
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;

            // um << ax, ay, 0.0;
            // if (az != 0)
            //   continue;

            inputs.push_back(um);
          }

      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
      {
        durations.push_back(tau);
      }
    }

    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        Eigen::Vector3d um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);

        Eigen::Vector3d pro_pos = pro_state.head(3);
        Eigen::Vector3i pro_id = PosToIndex(pro_pos);
        double pro_t = cur_node->time + tau;
        int pro_t_id = TimeToIndex(pro_t);

        // for log
        all_motions_.push_back(std::make_pair(cur_state, Eigen::Vector4d{um(0), um(1), um(2), tau}));

        // Check if in close set
        HybirdAstarPathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == NodeState::IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // Check maximal velocity
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;

        for (int k = 1; k <= check_num_; ++k)
        {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);

          if (isOccupied(pos, min_dist_))
          {
            is_occ = true;
            break;
          }
        }

        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = ((cost_axis_weight_ * um).squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal, max_vel_);

        // Compare nodes expanded from the same cameFrom
        // 检查新状态有无落在已经遍历过的体素中
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          HybirdAstarPathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        // 如果新状态落在了一个新的体素中
        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->cameFrom = cur_node;
            pro_node->node_state = NodeState::IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = TimeToIndex(pro_node->time);
            }

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            open_set_.push(pro_node);
            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            if (use_node_num_ == allocate_num_)
            {
              cout << "[hybird astar]: run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == NodeState::IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->cameFrom = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "[hybird astar]: error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  if (verbose_)
  {
    std::cout << "[hybird astar]: open set empty, no path!" << std::endl;
    std::cout << "[hybird astar]: use node num: " << use_node_num_ << std::endl;
    std::cout << "[hybird astar]: iter num: " << iter_num_ << std::endl;
  }

  return NO_PATH;
}

void HybirdAstar::retrievePath(HybirdAstarPathNodePtr end_node, std::vector<HybirdAstarPathNodePtr> &path_nodes)
{
  HybirdAstarPathNodePtr cur_node = end_node;
  path_nodes.push_back(cur_node);

  while (cur_node->cameFrom != NULL)
  {
    cur_node = cur_node->cameFrom;
    path_nodes.push_back(cur_node);
  }

  reverse(path_nodes.begin(), path_nodes.end());
}

bool HybirdAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal, Eigen::MatrixXd &coef_shot, double &t_shot)
{
  // x = a^3*t^3 + b^2*t^2 + c*t + d;
  /* ---------- get coefficient ---------- */
  const Eigen::Vector3d p0 = state1.head(3);
  const Eigen::Vector3d dp = state2.head(3) - p0;
  const Eigen::Vector3d v0 = state1.segment(3, 3);
  const Eigen::Vector3d v1 = state2.segment(3, 3);
  const Eigen::Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  Eigen::MatrixXd coef(3, 4);
  end_vel_ = v1;

  double v_max = max_vel_ * 0.2;
  t_d = dp.norm() / v_max;

  // Eigen::Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  // Eigen::Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Eigen::Vector3d a = (2*t_d*v0-2*dp+t_d*dv)/(t_d*t_d*t_d);
  Eigen::Vector3d b = (3*dp-3*v0*t_d-dv*t_d)/(t_d*t_d);
  Eigen::Vector3d c = v0;
  Eigen::Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  // Eigen::Vector3d coord, vel, acc;
  // Eigen::VectorXd poly1d, t, polyv, polya;
  // Eigen::Vector3i index;

  // Eigen::MatrixXd Tm(4, 4);
  // Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  // /* ---------- forward checking of trajectory ---------- */
  // double t_delta = t_d / 10;
  // for (double time = t_delta; time <= t_d; time += t_delta)
  // {
  //   t = VectorXd::Zero(4);
  //   for (int j = 0; j < 4; j++)
  //     t(j) = pow(time, j);

  //   for (int dim = 0; dim < 3; dim++)
  //   {
  //     poly1d = coef.row(dim);
  //     coord(dim) = poly1d.dot(t);
  //     vel(dim) = (Tm * poly1d).dot(t);
  //     acc(dim) = (Tm * Tm * poly1d).dot(t);

  //     if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
  //     {
  //       // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
  //       // return false;
  //     }
  //   }
  // }
  
  coef_shot = coef;
  t_shot = t_d;
  return true;
}

std::vector<double> HybirdAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

std::vector<double> HybirdAstar::quarticRoots(double a, double b, double c, double d, double e)
{
  // ax^4 + bx^3 + cx^2 + dx + e = 0
  // 四次方程的费拉里解法，https://zh.wikipedia.org/wiki/%E5%9B%9B%E6%AC%A1%E6%96%B9%E7%A8%8B，降次为三次方程电泳cubic
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

double HybirdAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time, double max_vel)
{
  // // 欧几里得距离
  // optimal_time = 1;
  // return (x2.head(3) - x1.head(3)).norm();

  // 曼哈顿距离
  // optimal_time = 1;
  // Eigen::Vector3i start = PosToIndex(Eigen::Vector3d{x1(0),x1(1),x1(2)});
  // Eigen::Vector3i end = PosToIndex(Eigen::Vector3d{x2(0),x2(1),x2(2)});
  // return double(std::abs(start[0]-end[0]) + std::abs(start[1]-end[1]) + std::abs(start[2]-end[2]));

  // J = \int u^2 dt + \rho T = -c1/(3*T^3) - c2/(2*T^2) - c3/T + w_time_*T;

  Eigen::Vector3d dp = x2.head(3) - x1.head(3);
  Eigen::Vector3d v0 = x1.segment(3, 3);
  Eigen::Vector3d v1 = x2.segment(3, 3);

  dp = cost_axis_weight_ * dp;
  v0 = cost_axis_weight_ * v0;
  v1 = cost_axis_weight_ * v1;

  double c0 = -36 * dp.dot(dp);
  double c1 = 24 * (v0 + v1).dot(dp);
  double c2 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c3 = 0;
  double c4 = w_time_;

  std::vector<double> ts = quarticRoots(c4, c3, c2, c1, c0);

  double v_max = max_vel * 0.5;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

Eigen::Vector3i HybirdAstar::PosToIndex(Eigen::Vector3d pt)
{
  Eigen::Vector3i idx = (pt * inv_resolution_).array().floor().cast<int>();

  return idx;
}

int HybirdAstar::TimeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void HybirdAstar::stateTransit(Eigen::Matrix<double, 6, 1> &state0, Eigen::Matrix<double, 6, 1> &state1, Eigen::Vector3d um, double tau)
{
  Eigen::Matrix<double, 6, 6> phi = Eigen::Matrix<double, 6, 6>::Identity();

  for (int i = 0; i < 3; ++i)
    phi(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi * state0 + integral;
}

std::vector<Eigen::Vector3d> HybirdAstar::getKinoTraj(double delta_t)
{
  std::vector<Eigen::Vector3d> state_list;

  if (path_nodes_.size() < 1)
  {
    std::cout << "There are no path nodes!" << std::endl;
    return state_list;
  }

  /* ---------- get traj of searching ---------- */
  HybirdAstarPathNodePtr node = path_nodes_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;

  while (node->cameFrom != NULL)
  {
    Eigen::Vector3d um = node->input;
    double duration = node->duration;
    x0 = node->cameFrom->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, um, t);
      state_list.push_back(xt.head(3));
    }
    node = node->cameFrom;
  }
  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void HybirdAstar::saveTrjToTxt(double delta_t, std::string filename)
{
  std::ofstream output_file(filename);
  // 检查文件是否成功打开
  if (!output_file)
  {
    std::cerr << "无法打开文件" << std::endl;
    exit(0);
  }

  std::vector<Eigen::Vector3d> pos_list;
  std::vector<Eigen::Vector3d> vel_list;
  std::vector<Eigen::Vector3d> acc_list;

  /* ---------- get traj of searching ---------- */
  HybirdAstarPathNodePtr node = path_nodes_.back();
  Eigen::Matrix<double, 6, 1> x0, xt;

  while (node->cameFrom != NULL)
  {
    Eigen::Vector3d um = node->input;
    double duration = node->duration;
    x0 = node->cameFrom->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, um, t);
      pos_list.push_back(xt.head(3));
      vel_list.push_back(xt.tail(3));
      acc_list.push_back(node->input);
    }
    node = node->cameFrom;
  }
  reverse(pos_list.begin(), pos_list.end());
  reverse(vel_list.begin(), vel_list.end());
  reverse(acc_list.begin(), acc_list.end());

  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      pos_list.push_back(coord);
    }

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      time(0) = 0;
      for (int j = 1; j < 4; j++)
        time(j) = j * pow(t, j - 1);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      vel_list.push_back(coord);
    }

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      time(0) = 0;
      time(1) = 0;
      for (int j = 2; j < 4; j++)
        time(j) = j * (j - 1) * pow(t, j - 2);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      acc_list.push_back(coord);
    }
  }

  if (output_file.is_open())
  {
    for (int i = 0; i < pos_list.size(); i++)
      output_file << i * delta_t << "," << 0 << ","
                  << pos_list[i][0] << "," << pos_list[i][1] << "," << pos_list[i][2] << ","
                  << vel_list[i][0] << "," << vel_list[i][1] << "," << vel_list[i][2] << ","
                  << acc_list[i][0] << "," << acc_list[i][1] << "," << acc_list[i][2] << "\n";
  }
}

std::vector<HybirdAstarPathNodePtr> HybirdAstar::getHybirdAstarPathNodes()
{
  std::vector<HybirdAstarPathNodePtr> pathNode_list;

  HybirdAstarPathNodePtr node = path_nodes_.back();
  while (node->cameFrom != NULL)
  {
    pathNode_list.push_back(node);
    node = node->cameFrom;
  }
  reverse(pathNode_list.begin(), pathNode_list.end());

  return pathNode_list;
}

std::vector<HybirdAstarPathNodePtr> HybirdAstar::getVisitedNodes()
{
  std::vector<HybirdAstarPathNodePtr> pathNode_list;

  HybirdAstarPathNodePtr node = path_nodes_.back();
  while (node->cameFrom != NULL)
  {
    pathNode_list.push_back(node);
    node = node->cameFrom;
  }
  reverse(pathNode_list.begin(), pathNode_list.end()); // The first node is the start node.

  std::priority_queue<HybirdAstarPathNodePtr, std::vector<HybirdAstarPathNodePtr>, NodeComparator> temp_queue;
  temp_queue = open_set_;
  int num = temp_queue.size();
  for (int i = 0; i < num; i++)
  {

    pathNode_list.push_back(temp_queue.top());
    temp_queue.pop();
  }

  return pathNode_list;
}

std::vector<Eigen::Vector3d> HybirdAstar::getVisitedPath(double delta_t)
{
  std::vector<Eigen::Vector3d> state_list;

  Eigen::Matrix<double, 6, 1> x0, xt;
  Eigen::Vector3d um;
  double duration;

  HybirdAstarPathNodePtr node;

  node = path_nodes_.back();
  while (node->cameFrom != NULL)
  {
    um = node->input;
    duration = node->duration;
    x0 = node->cameFrom->state;
    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, um, t);
      state_list.push_back(xt.head(3));
    }
    node = node->cameFrom;
  }

  std::priority_queue<HybirdAstarPathNodePtr, std::vector<HybirdAstarPathNodePtr>, NodeComparator> temp_queue;
  temp_queue = open_set_;
  int num = temp_queue.size();
  for (int i = 0; i < num; i++)
  {
    node = temp_queue.top();
    temp_queue.pop();

    um = node->input;
    duration = node->duration;
    x0 = node->cameFrom->state;
    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, um, t);
      state_list.push_back(xt.head(3));
    }
  }

  return state_list;
}

std::vector<Eigen::Vector3d> HybirdAstar::getAllMotions(double delta_t)
{
  std::vector<Eigen::Vector3d> state_list;

  Eigen::Matrix<double, 6, 1> x0, xt;
  Eigen::Vector3d um;
  double duration;

  for (int i = 0; i < all_motions_.size(); i++)
  {
    um = Eigen::Vector3d{all_motions_[i].second[0], all_motions_[i].second[1], all_motions_[i].second[2]};
    duration = all_motions_[i].second[3];
    x0 = all_motions_[i].first;
    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, um, t);
      state_list.push_back(xt.head(3));
    }
  }

  return state_list;
}

// void HybirdAstar::parameterTraj(double start_time, double delta_t)
// {
//   int dim = 3;
//   std::vector<double> time_pts;
//   std::vector<Eigen::Vector3d> points = getKinoTraj(delta_t);
//   Eigen::MatrixXd wps = Eigen::MatrixXd::Zero(points.size(), 3);

//   for(int i=0; i<points.size(); i++)
//   {
//     wps.row(i) = points[i];
//     time_pts.push_back(start_time+delta_t*i);
//   }

//   HybirdAstarPathNodePtr node = path_nodes_.front();

//   Eigen::MatrixXd bb = Eigen::MatrixXd::Zero(4, dim);
//   // bb.row(0) = node->state.head(3);
//   // bb.row(1) = node->state.tail(3);
//   // std::cout<<node->state.head(3)<<" "<<node->state.tail(3)<<std::endl;
//   trajectory_.QuinticSplineInterpolation(dim, wps, time_pts, bb);
// }
