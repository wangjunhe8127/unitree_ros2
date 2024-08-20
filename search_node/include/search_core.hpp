#pragma once
#include "common.hpp"

namespace planning {
namespace path {
class SearchCore {
public:
  SearchCore();
  bool Process(const SearchIn &input, SearchOut &output) {}

private:
  void GenerateSearchMap(input);
}
} // namespace path
} // namespace planning