#pragma once
#include "collision_checker.hpp"
#include "common.hpp"
#include "search_node.hpp"
namespace unitree {
namespace planning {
double SearchNode::pos_resolution = 0.2;
double SearchNode::theta_resolution = 0.1;
double SearchNode::x_bias = 0.0;
double SearchNode::y_bias = 0.0;
double SearchNode::theta_bias = 0.0;
class SearchCore {
public:
  SearchCore() { collision_checker_ = std::make_shared<CollisonCheck>(); };
  virtual ~SearchCore() = default;
  void Init(const SearchIn &input) {
    search_in_ = input;
    std::cout << "loc_x:" << search_in_.loc_point.x << std::endl;
    std::cout << "loc_y:" << search_in_.loc_point.y << std::endl;
    std::cout << "heading:" << search_in_.loc_point.heading << std::endl;
    collision_checker_->Init(hybrid_a_star_config_);
    collision_checker_->SetPosBias(search_in_.map_origin_point);
    collision_checker_->SetSearchMap(search_in_);
    SearchNode::pos_resolution = hybrid_a_star_config_.pos_resolution;
    SearchNode::theta_resolution = hybrid_a_star_config_.theta_resolution;
    SearchNode::x_bias = search_in_.search_area.min_x;
    SearchNode::y_bias = search_in_.search_area.min_y;
    SearchNode::theta_bias = -M_PI;
  }
  bool Process(PlanningResult &planning_result) {

    start_node_ = std::make_shared<SearchNode>(
      std::vector<double>({search_in_.loc_point.x}),
      std::vector<double>({search_in_.loc_point.y}),
      std::vector<double>({search_in_.loc_point.heading}));
    end_node_ = std::make_shared<SearchNode>(
      std::vector<double>({search_in_.ref_path.back().x}),
      std::vector<double>({search_in_.ref_path.back().y}),
      std::vector<double>({search_in_.ref_path.back().heading}));
    open_set_.clear();
    close_set_.clear();
    open_pq_ = decltype(open_pq_)(); // use decltype to clear priority_queue
    final_node_ = nullptr;
    open_set_.emplace(start_node_->GetIndex(), start_node_);
    open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());
    static int times = 0;
    times++;
    // hybrid a start algo core
    size_t explored_times = 0;
    int tmp_size = 0;
    while (!open_pq_.empty()) {
      // check max search times
      if (explored_times > hybrid_a_star_config_.max_search_times) {
        break;
      }
      // get the best node
      auto current_id = open_pq_.top().first;
      std::shared_ptr<SearchNode> current_node = open_set_[current_id];
      close_set_.emplace(current_node->GetIndex(), current_node);
      open_pq_.pop();
      // explore
      for (size_t i = 0; i < hybrid_a_star_config_.node_explore_size; i++) {
        std::shared_ptr<SearchNode> next_node;
        double delta_steer = hybrid_a_star_config_.max_steer_anglerate *
                             hybrid_a_star_config_.node_dt;
        next_node = SearchNodeExplore(
            current_node, delta_steer * (static_cast<double>(i % 5) - 2));
        if (next_node == nullptr) {
          continue;
        }
        if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
          continue;
        }
        if (!ValidityCheck(next_node)) {
          continue;
        }
        if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
          explored_times++;
          CalculateNodeCost(current_node, next_node);
          open_set_.emplace(next_node->GetIndex(), next_node);
          open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
          if (ReachedEndCheck(next_node)) {
            final_node_ = next_node;
            break;
          }
        }
        tmp_size++;
      }
  
      if (final_node_ != nullptr) {
        break;
      }
  }
      if (!GetPath(planning_result)) {
        std::cout << "end_record" << std::endl;
      }
      if (!GetTrajectory(planning_result)) {
        std::cout << "end_record" << std::endl;
        return false;
      }
      return true;
    }
bool ReachedEndCheck(std::shared_ptr<SearchNode> node) {
  double delta_theta =
      NormalizeAngle(end_node_->theta - node->theta);

  if (std::fabs(delta_theta) > hybrid_a_star_config_.finish_max_delta_theta) {
    return false;
  }
  double dx = node->x - end_node_->x;
  double dy = node->y - end_node_->y;
  double distance = std::sqrt(dx*dx+dy*dy);
  if (distance > hybrid_a_star_config_.finish_max_delta_distance){
    return false;
  }
  return true;
}
bool GetTrajectory(PlanningResult &planning_result) {
  double s = 0.0;
  if (static_cast<int>(planning_result.size()) <= 1) {
    std::cout<<"planning_result_size:" << planning_result.size() << std::endl;
    planning_result.clear();
    PlanningPoint point;
    point.s = 0.0;
    point.t = 0.0;
    point.x = search_in_.loc_point.x;
    point.y = search_in_.loc_point.y;
    point.yaw = search_in_.loc_point.heading;
    point.vx = 0.0;
    point.vy = 0.0;
    point.vyaw = 0.0;
    planning_result.push_back(point);
    return true;
  }
  std::cout<<"planning_result_size1:" << planning_result.size() << std::endl;
  std::vector<double> s_arr;
  s_arr.push_back(0.0);
  for (int i = 1; i < static_cast<int>(planning_result.size()); i++) {
    double dx = planning_result.at(i).x - planning_result.at(i - 1).x;
    double dy = planning_result.at(i).y - planning_result.at(i - 1).y;
    s += std::hypot(dx, dy);
    s_arr.push_back(s);
  }
  std::cout<<"s_arr.size():" << s_arr.size() << std::endl;
  // TODO(wjh) optimal trajectory
  //***********************当前方案************************/
  // 先根据s，v,max_v,a等信息计算t
  // 再根据t/30得到每一个点的时间间隔
  // 然后根据curr_t，反查所在的运动段（加速/匀速/减速），并获得当前的s，同时获得插值点，以及对每个点赋值对应的速度信息
  // vyaw直接由前后点的heading_rate得到

  double start_v = search_in_.loc_point.v;
  double critical_len;
  double run_time;
  if (start_v > max_v) {
    start_v = max_v;
  }
  double startv2 = pow(start_v, 2);
  double endv2 = pow(end_v, 2);
  double maxv2 = pow(max_v, 2);
  double start_acc_len = (maxv2 - startv2) / (2 * max_acc);
  double stop_acc_len = (maxv2 - endv2) / (2 * max_acc);
  critical_len = start_acc_len + stop_acc_len;
  if (s_arr.back() >= critical_len) {
    run_time = (max_v - start_v) / max_acc + (max_v - end_v) / max_acc +
           (s_arr.back() - critical_len) / max_v;
  } else {
    double tmpv = sqrt(0.5 * (startv2 + endv2 + 2 * max_acc * s_arr.back()));
    run_time = (tmpv - start_v) / max_acc + (tmpv - end_v) / max_acc;
  }
  std::cout<<"start_v:" << start_v <<std::endl;
  std::cout<<"start_acc_len:" << start_acc_len <<std::endl;
  std::cout<<"stop_acc_len:" << stop_acc_len <<std::endl;
  std::cout<<"critical_len:" << critical_len <<std::endl;
  std::cout<<"plan_t:" << run_time <<std::endl;
  std::cout<<"plan_s:" << s_arr.back() << std::endl;
  double unit_time = run_time / 30.0;
  double curr_t = 0.0;
  PlanningResult new_planning_result;
  for (int i = 0; i < 30; i++) {
    double curr_v, curr_s,curr_yaw, curr_x, curr_y;
    curr_t += unit_time; // 不是从t=0开始
    PlanningPoint planning_point;
    GetVandS(planning_result, s_arr, curr_t, curr_s, curr_v);
    GetPointByS(planning_result, curr_s, s_arr, curr_yaw, curr_x, curr_y);
    planning_point.t = curr_t;
    planning_point.s = curr_s;
    planning_point.x = curr_x;
    planning_point.y = curr_y;
    planning_point.yaw = curr_yaw;
    planning_point.vx = curr_v * std::cos(curr_yaw);
    planning_point.vy = curr_v * std::sin(curr_yaw);
    new_planning_result.push_back(planning_point);
  }
  planning_result = new_planning_result;
  // 计算vyaw
  for (int i = 0; i < 29; i++) {
    planning_result.at(i).vyaw = planning_result.at(i + 1).vyaw - planning_result.at(i).vyaw;
  }
  planning_result.at(29).vyaw = 0.0;
  return true;
}
  void GetVandS(const PlanningResult &planning_result, const std::vector<double> &s_arr, const double &curr_t, double &curr_s, double &curr_v) {
    double start_v = search_in_.loc_point.v;
    double critical_len;
  if (start_v > max_v) {
    start_v = max_v;
  }
  double startv2 = pow(start_v, 2);
  double endv2 = pow(end_v, 2);
  double maxv2 = pow(max_v, 2);
  double start_acc_len = (maxv2 - startv2) / (2 * max_acc);
  double stop_acc_len = (maxv2 - endv2) / (2 * max_acc);
  critical_len = start_acc_len + stop_acc_len;
  double locallength = s_arr.back();
  if (locallength >= critical_len) {
    std::cout << "critical_len: " << critical_len <<std::endl;
    std::cout << "locallength: " << locallength <<std::endl;
    double t1 = (max_v - start_v) / max_acc;
    double t2 = t1 + (locallength - critical_len) / max_v;
    std::cout << "t1: " << t1 <<std::endl;
    std::cout << "t2: " << t2 <<std::endl;
    std::cout << "curr_t: " << curr_t <<std::endl;
    if (curr_t <= t1) {
      curr_v = start_v + max_acc * curr_t;
      curr_s = start_v * curr_t + 0.5 * max_acc * pow(curr_t, 2);
      std::cout << "curr_v5: " << curr_v <<std::endl;
      std::cout << "curr_s: " << curr_s <<std::endl;
      return ;
    } else if (curr_t <= t2) {
      curr_v = start_v + max_acc * t1;
      curr_s = start_v * t1 + 0.5 * max_acc * pow(t1, 2) + (curr_t - t1) * max_v;
      std::cout << "curr_v4: " << curr_v <<std::endl;
      std::cout << "curr_s: " << curr_s <<std::endl;
      return ;
    } else {
      curr_v = start_v + max_acc * t1 - max_acc * (curr_t - t2);
      curr_s = start_v * t1 + 0.5 * max_acc * pow(t1, 2) + (t2 - t1) * max_v +
             max_v * (curr_t - t2) - 0.5 * max_acc * pow(curr_t - t2, 2);
      std::cout << "curr_v3: " << curr_v <<std::endl;
      std::cout << "curr_s: " << curr_s <<std::endl;
      return ;
    }
  } else {
    double tmpv = sqrt(0.5 * (startv2 + endv2 + 2 * max_acc * locallength));
    double tmpt = (tmpv - start_v) / max_acc;
    if (curr_t <= tmpt) {
      curr_v = start_v + max_acc * curr_t;
      curr_s = start_v * curr_t + 0.5 * max_acc * pow(curr_t, 2);
      std::cout << "curr_v2: " << curr_v <<std::endl;
      std::cout << "curr_s: " << curr_s <<std::endl;
      return ;
    } else {
      curr_v = tmpv + max_acc + (curr_t - tmpt);
      curr_s = start_v * tmpt + 0.5 * max_acc * pow(tmpt, 2) +
               tmpv * (curr_t - tmpt) - 0.5 * max_acc * pow(curr_t - tmpt, 2);
      std::cout << "curr_t: " << curr_t <<std::endl;
      std::cout << "start_v: " << start_v <<std::endl;
      std::cout << "tmpv: " << tmpv <<std::endl;
      std::cout << "tmpt: " << tmpt <<std::endl;
      std::cout << "curr_s: " << curr_s <<std::endl;
      std::cout << "curr_v1: " << curr_v <<std::endl;
      std::cout << "curr_s: " << curr_s <<std::endl;
      return ;
    }
  }
  }
  void GetPointByS(const PlanningResult &planning_result, double curr_s, const std::vector<double> &s_arr, double &curr_yaw, double &curr_x, double &curr_y) {
    int index = 0;
    for (index = 0; index < static_cast<int>(s_arr.size()); index++) {
        if (s_arr[index] + 0.00000001 >= curr_s) {
            break;
        }
    }
      std::cout << "s_arr.size():" << s_arr.size() << std::endl;
      std::cout << "curr_s:" << curr_s << std::endl;
      std::cout << "index2:" << index << std::endl;
      std::cout << "s_arr.back:" << s_arr.back() << std::endl;
    if (index < static_cast<int>(s_arr.size()) && s_arr[index] == curr_s) {
      std::cout << "s_arr.size():" << s_arr.size() << std::endl;
      std::cout << "curr_s:" << curr_s << std::endl;
      std::cout << "index0:" << index << std::endl;
      std::cout << "s_arr.back:" << s_arr.back() << std::endl;
      curr_yaw = planning_result.at(index).yaw;
      curr_x = planning_result.at(index).x;
      curr_y = planning_result.at(index).y;
      return;
    }
    // 这个情况不会出现，会在得到s的函数里保证
    if (index == 0 || index == static_cast<int>(s_arr.size())) {
      std::cout << "s_arr.size():" << s_arr.size() << std::endl;
      std::cout << "curr_s:" << curr_s << std::endl;
      std::cout << "index1:" << index << std::endl;
      std::cout << "s_arr.back:" << s_arr.back() << std::endl;
        throw std::out_of_range("curr_s is out of the bounds of the s array");
    }
    // 执行线性插值
    double s1 = s_arr.at(index - 1);
    double s2 = s_arr.at(index);
    double x1 = planning_result.at(index - 1).x;
    double x2 = planning_result.at(index).x;
    double y1 = planning_result.at(index - 1).y;
    double y2 = planning_result.at(index).y;
    double x_interp = x1 + (x2 - x1) * (curr_s - s1) / (s2 - s1);
    double y_interp = y1 + (y2 - y1) * (curr_s - s1) / (s2 - s1);
    curr_yaw = planning_result.at(index - 1).yaw;
    curr_x = x_interp;
    curr_y = y_interp;
      std::cout << "s_arr.size():" << s_arr.size() << std::endl;
      std::cout << "curr_s:" << curr_s << std::endl;
      std::cout << "index0:" << index << std::endl;
      std::cout << "s_arr.back:" << s_arr.back() << std::endl;
      std::cout << "planning_result.at(index - 1).x:" << planning_result.at(index - 1).x << std::endl;
      std::cout << "planning_result.at(index - 1).y:" << planning_result.at(index - 1).y << std::endl;
  }
  bool GetPath(PlanningResult &planning_result) {
    std::shared_ptr<SearchNode> current_node = final_node_;
    std::vector<double> hybrid_a_x;
    std::vector<double> hybrid_a_y;
    std::vector<double> hybrid_a_theta;
    while (current_node != nullptr) {
      std::vector<double> x = current_node->GetSeqX();
      std::vector<double> y = current_node->GetSeqY();
      std::vector<double> theta = current_node->GetSeqTheta();
      if (x.size() != y.size() || x.size() != theta.size() || x.empty()) {
        return false;
      }
      std::reverse(x.begin(), x.end());
      std::reverse(y.begin(), y.end());
      std::reverse(theta.begin(), theta.end());
      hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
      hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
      hybrid_a_theta.insert(hybrid_a_theta.end(), theta.begin(), theta.end());
      current_node = current_node->GetPreNode();
    }
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_theta.begin(), hybrid_a_theta.end());
    planning_result.clear();
    // init gear
    for (size_t i = 0; i < hybrid_a_x.size(); i++) {
      std::cout<<"search_x:" << hybrid_a_x[i]<<std::endl;
      std::cout<<"search_y:" << hybrid_a_y[i]<<std::endl;
      PlanningPoint planning_point;
      planning_point.x = hybrid_a_x[i];
      planning_point.y = hybrid_a_y[i];
      planning_point.yaw = hybrid_a_theta[i];
      planning_result.push_back(planning_point);
    }
    return true;
  }
  std::shared_ptr<SearchNode>
  SearchNodeExplore(std::shared_ptr<SearchNode> current_node,
                    double delta_steer) {
    double steer_explored = current_node->steering;
    double max_steer_angle = hybrid_a_star_config_.max_steer_angle; // in rad
    steer_explored = Clamp(steer_explored + delta_steer,
                                         -max_steer_angle, max_steer_angle);
    double search_step_s = hybrid_a_star_config_.search_step_s;
    double wheelbase = hybrid_a_star_config_.wheel_base;
    // node explore
    std::vector<double> x_sequence;
    std::vector<double> y_sequence;
    std::vector<double> theta_sequence;
    double start_x = current_node->x;
    double start_y = current_node->y;
    double start_theta = current_node->theta;
    x_sequence.push_back(start_x);
    y_sequence.push_back(start_y);
    theta_sequence.push_back(start_theta);
    // RK2
    double next_x = start_x + search_step_s / 2 * std::cos(start_theta);
    double next_y = start_y + search_step_s / 2 * std::sin(start_theta);
    double next_theta =
        start_theta + search_step_s / wheelbase / 2 * std::tan(steer_explored);
    next_x += search_step_s / 2 * std::cos(next_theta);
    next_y += search_step_s / 2 * std::sin(next_theta);
    next_theta = NormalizeAngle(
        next_theta + search_step_s / wheelbase / 2 * std::tan(steer_explored));
    x_sequence.push_back(next_x);
    y_sequence.push_back(next_y);
    theta_sequence.push_back(next_theta);
    start_x = next_x;
    start_y = next_y;
    start_theta = next_theta;
    // check if the vehicle runs outside of XY boundary
    if (x_sequence.back() > search_in_.search_area.max_x ||
        x_sequence.back() < search_in_.search_area.min_x ||
        y_sequence.back() > search_in_.search_area.max_y ||
        y_sequence.back() < search_in_.search_area.min_y) {
      return nullptr;
    }
    std::shared_ptr<SearchNode> next_node = std::shared_ptr<SearchNode>(
        new SearchNode(x_sequence, y_sequence, theta_sequence));
    next_node->SetPreNode(current_node);
    next_node->steering = steer_explored;
    next_node->depth = current_node->depth + 1;
    return next_node;
  }
  bool ValidityCheck(std::shared_ptr<SearchNode> node) {
    auto sequence_x = node->GetSeqX();
    auto sequence_y = node->GetSeqY();
    auto squence_theta = node->GetSeqTheta();
    auto roi_boundary = search_in_.search_area;
    for (int i = 0; i < sequence_x.size(); i++) {
      if (sequence_x[i] > roi_boundary.max_x ||
          sequence_x[i] < roi_boundary.min_x ||
          sequence_y[i] > roi_boundary.max_y ||
          sequence_y[i] < roi_boundary.min_y) {
        return false;
      }
      if (!collision_checker_->ValidityCheck(sequence_x[i], sequence_y[i],
                                             squence_theta[i])) {
        return false;
      }
    }
    return true;
  }
  void CalculateNodeCost(std::shared_ptr<SearchNode> current_node,
                         std::shared_ptr<SearchNode> next_node) {
    // caculate path cost
    double path_cost =
        current_node->path_cost + PathCost(current_node, next_node);

    // caculate reference line cost
    double refline_g_cost = 0.0;
    double refline_h_cost = 0.0;
    ReferenceLineCost(next_node, refline_g_cost, refline_h_cost);

    path_cost += refline_g_cost;
    next_node->path_cost = path_cost;
    next_node->ref_line_cost = refline_g_cost;
    next_node->heuristic_cost = refline_h_cost;
  }
  double PathCost(std::shared_ptr<SearchNode> current_node,
                  std::shared_ptr<SearchNode> next_node) {
    double piecewise_cost = 0.0;
    double node_s = hybrid_a_star_config_.search_step_s;
    piecewise_cost += node_s * hybrid_a_star_config_.cost_weight;
    piecewise_cost +=
        hybrid_a_star_config_.cost_steer * std::abs(next_node->steering);
    piecewise_cost += hybrid_a_star_config_.cost_steerrate *
                      std::abs(next_node->steering - current_node->steering);
    return piecewise_cost;
  }
  void ReferenceLineCost(std::shared_ptr<SearchNode> node,
                         double &g_cost,   // NOLINT
                         double &h_cost) { // NOLINT
    double x = node->x;
    double y = node->y;
    double min_l = 100.0;
    int min_index = 0;
    for (int i = 0; i < search_in_.ref_path.size(); i++) {
      const double dx = search_in_.ref_path[i].x - x;
      const double dy = search_in_.ref_path[i].y - y;
      double l = std::hypot(dx, dy);
      if (l < min_l) {
        min_l = l;
        min_index = i;
      }
    }
    double refline_delta_theta =
        NormalizeAngle(node->theta - search_in_.ref_path[min_index].heading);
    g_cost = 0;
    int preview_index =
        std::min(static_cast<int>(search_in_.ref_path.size() - 1),
                 min_index + hybrid_a_star_config_.preiew_length);
    auto preview_point = search_in_.ref_path[preview_index];
    double h_e_distance =
        std::hypot(preview_point.x - x, preview_point.y - y);
    double h_s_distance =std::hypot(search_in_.ref_path.back().x - x, search_in_.ref_path.back().y - y);
    double ref_left_distance = h_e_distance + h_s_distance;
    double h_heading_cost = NormalizeAngle(
        node->theta - std::atan2(preview_point.y - y, preview_point.x - x));
    double h_attract_cost = h_e_distance + std::fabs(h_heading_cost) / 3.0;
    h_cost =
        hybrid_a_star_config_.cost_refline_h * (h_s_distance + h_attract_cost);
  }

private:
  std::shared_ptr<CollisonCheck> collision_checker_;
  HybirdAStarConfig hybrid_a_star_config_;
  typedef std::pair<std::string, double> OpenPQType;
  struct pq_cmp {
    bool operator()(const OpenPQType& left, const OpenPQType& right) const {
      return left.second >= right.second;
    }
  };
  double max_acc = 0.2;
  double max_v = 0.6;
  double end_v = 0.0; // 这里需要注意，每次搜索不是到当前路线的航点结束，而是将搜索终点局限在hight_map的区域内[routing只发了这一段&&search grid限制]
  SearchIn search_in_;
  std::shared_ptr<SearchNode> start_node_;
  std::shared_ptr<SearchNode> end_node_;
  std::shared_ptr<SearchNode> final_node_;
  std::priority_queue<OpenPQType, std::vector<OpenPQType>, pq_cmp> open_pq_;
  std::unordered_map<std::string, std::shared_ptr<SearchNode>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<SearchNode>> close_set_;
};
} // namespace planning
} // namespace unitree