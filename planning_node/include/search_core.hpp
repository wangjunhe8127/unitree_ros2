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
  bool Process(SearchOut &output) {

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
      if (!GetResult()) {
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
  bool GetResult() {
    std::shared_ptr<SearchNode> current_node = final_node_;
    std::vector<double> hybrid_a_x;
    std::vector<double> hybrid_a_y;
    std::vector<double> hybrid_a_theta;
    while (current_node != nullptr) {
      std::vector<double> x = current_node->GetSeqX();
      std::vector<double> y = current_node->GetSeqY();
      std::vector<double> theta = current_node->GetSeqTheta();
      if (x.size() != y.size() || x.size() != theta.size() || x.empty()) {
        std::cout<< "33" << std::endl;
        return false;
      }
      std::cout<< "34" << std::endl;
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
    path_result_.clear();
    path_result_.emplace_back();
    SearchSegmentResult *current_path = &(path_result_.back());
    double min_replacement = 0.01;
    // init gear
    for (size_t i = 0; i < hybrid_a_x.size(); i++) {
      std::cout<<"search_x:" << hybrid_a_x[i]<<std::endl;
      std::cout<<"search_y:" << hybrid_a_y[i]<<std::endl;
      current_path->x.push_back(hybrid_a_x[i]);
      current_path->y.push_back(hybrid_a_y[i]);
      current_path->theta.push_back(hybrid_a_theta[i]);
    }
    std::cout << "end_record" << std::endl;
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
  SearchIn search_in_;
  std::vector<SearchSegmentResult> path_result_;
  std::shared_ptr<SearchNode> start_node_;
  std::shared_ptr<SearchNode> end_node_;
  std::shared_ptr<SearchNode> final_node_;
  std::priority_queue<OpenPQType, std::vector<OpenPQType>, pq_cmp> open_pq_;
  std::unordered_map<std::string, std::shared_ptr<SearchNode>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<SearchNode>> close_set_;
};
} // namespace planning
} // namespace unitree