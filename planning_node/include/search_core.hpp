#pragma once
#include "common.hpp"
#include "collision_checker.hpp"
namespace unitree {
namespace planning {
class SearchCore {
public:
  SearchCore() {
    collision_checker_ = std::make_shared<CollisonCheck>();
  };
  virtual ~SearchCore() = default;
  void Init(SearchIn &input) {
    std::cout << "loc_x:" << input.loc_point.x<< std::endl;
    std::cout << "loc_y:" << input.loc_point.y<< std::endl;
    std::cout << "heading:" << input.loc_point.yaw<< std::endl;
    collision_checker_->Init(hybrid_a_star_config_);
    collision_checker_->SetPosBias(input.map_origin_point);
    collision_checker_->SetSearchMap(input);
  }
  // bool Process(const SearchIn &input, SearchOut &output) { }

private:
  std::shared_ptr<CollisonCheck> collision_checker_;
  HybirdAStarConfig hybrid_a_star_config_;
};
} // namespace path
} // namespace planning