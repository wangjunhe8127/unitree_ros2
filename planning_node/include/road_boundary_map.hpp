#pragma once
#include "common.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
namespace std {
template <> struct hash<std::pair<int, int>> {
  size_t operator()(const std::pair<int, int> &p) const {
    size_t hash1 = std::hash<int>{}(p.first);
    size_t hash2 = std::hash<int>{}(p.second);
    return hash1 ^ (hash2 << 1); // Combine the two hash values
  }
};
} // namespace std
namespace unitree {
namespace planning {

class RoadBoundaryMap {
public:
  RoadBoundaryMap() = default;
  virtual ~RoadBoundaryMap() = default;
  void Init(const HybirdAStarConfig &config) {
    pos_resolution_ = config.pos_resolution;
    expand_radius_ = config.expand_radius;
    int search_size =
        static_cast<int>(expand_radius_ / (pos_resolution_ / 2.0));
    double search_step = pos_resolution_ / 2.0;
    expand_circle_.clear();
    for (int i = -search_size; i <= search_size; i++) {
      for (int j = -search_size; j <= search_size; j++) {
        double circle_x = i * search_step;
        double circle_y = j * search_step;
        if (std::hypot(circle_x, circle_y) <= expand_radius_) {
          expand_circle_.emplace_back(circle_x, circle_y);
        }
      }
    }
  }
  void SetBoundary(const SearchIn &search_in) {
    for (int i = 0; i < static_cast<int>(search_in.left_path.size()); i++) {
      SetMapPoint(search_in.left_path.at(i).x, search_in.left_path.at(i).y);
    }
    for (int i = 0; i < static_cast<int>(search_in.right_path.size()); i++) {
      SetMapPoint(search_in.right_path.at(i).x, search_in.right_path.at(i).y);
    }
  }
  void SetHightMapPoint(const SearchIn &search_in) {
    for (int i = 0; i < static_cast<int>(search_in.map_data.size()); i++) {
      if (search_in.map_data.at(i)[2] > 0.25) {
        SetMapPoint(search_in.map_data.at(i)[0], search_in.map_data.at(i)[1]);
      }
    }
  }

  void SetMapPoint(double x, double y) {
    for (auto point : expand_circle_) {
      double x_in_circle = point.first + x;
      double y_in_circle = point.second + y;
      boundary_map_.insert(GetPosEncoding(x_in_circle, y_in_circle));
    }
  }
  void ClearBoundary() { boundary_map_.clear(); }
  void SetXBias(double bias) { x_bias_boundary_map_ = bias; }
  void SetYBias(double bias) { y_bias_boundary_map_ = bias; }
  bool CollisionCheck(double x, double y) {
    auto pos_encoding = GetPosEncoding(x, y);
    if (boundary_map_.find(pos_encoding) != boundary_map_.end()) {
      return true;
    } else {
      return false;
    }
  }
  void GetMapLog() {
    for (auto point : boundary_map_) {
      std::cout << "boundary_map_x:"
                << point.first * pos_resolution_ + x_bias_boundary_map_
                << std::endl;
      std::cout << "boundary_map_y:"
                << point.second * pos_resolution_ + y_bias_boundary_map_
                << std::endl;
    }
  }

private:
  std::pair<int, int> GetPosEncoding(double x, double y) {
    int x_idx = static_cast<int>((x - x_bias_boundary_map_) / pos_resolution_);
    int y_idx = static_cast<int>((y - y_bias_boundary_map_) / pos_resolution_);
    return std::make_pair(x_idx, y_idx);
  }

  // road boundary map config
  double pos_resolution_{0.2};
  double expand_radius_{1.0};
  // set bounday map bias
  double x_bias_boundary_map_{0.0};
  double y_bias_boundary_map_{0.0};

  // boundary map data struct
  std::unordered_set<std::pair<int, int>> boundary_map_;
  std::vector<std::pair<double, double>> expand_circle_;
};

} // namespace planning
} // namespace unitree
