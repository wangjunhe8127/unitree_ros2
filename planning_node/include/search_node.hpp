
#pragma once
#include <memory>
#include <string>
#include <vector>

namespace unitree {
namespace planning {

class SearchNode {
public:
  explicit SearchNode(const std::vector<double> &x_seq,
                      const std::vector<double> &y_seq,
                      const std::vector<double> &theta_seq) {
    // init x,y,theta
    x = x_seq.back();
    y = y_seq.back();
    theta = theta_seq.back();
    x_seq_ = x_seq;
    y_seq_ = y_seq;
    theta_seq_ = theta_seq;
    seq_size = x_seq.size();
    x_grid = static_cast<int>((x - x_bias) / pos_resolution);
    y_grid = static_cast<int>((y - y_bias) / pos_resolution);
    theta_grid = static_cast<int>((theta - theta_bias) / theta_resolution);
    index_ = ComputeStringIndex(x_grid, y_grid, theta_grid);
  }

  virtual ~SearchNode() = default;

  double GetCost() const { return path_cost + heuristic_cost; }

  const std::string &GetIndex() const { return index_; }

  int GetSameGearCount() const { return same_gear_count; }

  std::shared_ptr<SearchNode> GetPreNode() const { return previous_node_; }

  void SetPreNode(std::shared_ptr<SearchNode> previous_node) {
    previous_node_ = previous_node;
  }

  const std::vector<double> &GetSeqX() const { return x_seq_; }

  const std::vector<double> &GetSeqY() const { return y_seq_; }

  const std::vector<double> &GetSeqTheta() const { return theta_seq_; }

private:
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
    return std::to_string(x_grid) + "_" + std::to_string(y_grid) + "_" +
           std::to_string(phi_grid);
  }

public:
  // static config
  static double pos_resolution;
  static double theta_resolution;
  static double x_bias;
  static double y_bias;
  static double theta_bias;
  // node parameters
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double steering{0.0};
  int x_grid{0};
  int y_grid{0};
  int theta_grid{0};
  bool is_forward{true};
  int depth{0};
  int seq_size{1};
  int gear_switch_count{0};
  int same_gear_count{0};
  // node cost
  double path_cost{0.0};
  double heuristic_cost{0.0};
  double ref_line_cost{0.0};
  double virtual_collision_cost{-1.0};

private:
  std::vector<double> x_seq_;
  std::vector<double> y_seq_;
  std::vector<double> theta_seq_;
  std::string index_;
  std::shared_ptr<SearchNode> previous_node_{nullptr};
};

} // namespace planning
} // namespace unitree