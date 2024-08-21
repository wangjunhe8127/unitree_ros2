#pragma once
#include "common.hpp"

namespace planning {
namespace path {
class SearchCore {
public:
  SearchCore()=default;
  virtual ~SearchCore() = default;
  void Init(SearchIn &input) {
    collision_checker_->Init(hybrid_a_star_config_);
    collision_checker_->SetPosBias(input.map_origin_point);
    collision_checker_->SetSearchMap(input);
  }
  // bool Process(const SearchIn &input, SearchOut &output) { }

private:
  std::shared_ptr<CollisonCheck> collision_checker_;
  HybirdAStarConfig hybrid_a_star_config_;
}
} // namespace path
} // namespace planning