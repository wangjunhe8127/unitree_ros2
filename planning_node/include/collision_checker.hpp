/**
 * @file collision_checker.h
 * @brief collison checker including AABB and SAT
 * @version 0.1
 * @date 2024-06-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "road_boundary_map.hpp"
#include <memory>
#include <string>
#include <vector>

namespace unitree {
namespace planning {

class CollisonCheck {
public:
  /**
   * @brief Construct a new Collison Check object
   *
   */
  CollisonCheck() { road_boundary_map_ = std::make_shared<RoadBoundaryMap>(); }

  /**
   * @brief Detory Collision Check
   *
   */
  virtual ~CollisonCheck() = default;

  /**
   * @brief init collision check
   *
   * @param config
   */
  void Init(const HybirdAStarConfig &config) {
    road_boundary_map_->Init(config);
  }
  void SetSearchMap(const SearchIn &search_in) {
    road_boundary_map_->ClearBoundary();
    road_boundary_map_->SetHightMapPoint(search_in);
    road_boundary_map_->SetBoundary(search_in);
    road_boundary_map_->GetMapLog();
  }
  /**
   * @brief set road boudary map bias
   *
   * @param x_bias
   * @param y_bias
   */
  void SetPosBias(const StatePoint &map_origin_point) {
    road_boundary_map_->SetXBias(map_origin_point.x);
    road_boundary_map_->SetYBias(map_origin_point.y);
  }

  bool ValidityCheck(double x, double y, double theta) {
    if (!RoadBoundaryValidityCheck(x, y, theta)) {
      return false;
    }
    return true;
  }

  bool RoadBoundaryValidityCheck(double x, double y, double theta) {
    for (int i = -1; i <= 1; i++) {
      // TODO(lkp): use right radius caculate formula
      double unitree_length = 0.7;
      double x_center = x + unitree_length / 3 * std::cos(theta) * i;
      double y_center = y + unitree_length / 3 * std::sin(theta) * i;
      if (road_boundary_map_->CollisionCheck(x_center, y_center)) {
        return false;
      }
    }
    return true;
  }

  std::shared_ptr<RoadBoundaryMap> road_boundary_map_;
};

} // namespace planning
} // namespace unitree
