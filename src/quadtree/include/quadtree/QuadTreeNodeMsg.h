#pragma once

#include <cstdint>

namespace QuadTree::Msg {

struct QuadTreeNodeMsg {
  int8_t val;
  int x, y;
  int size;
  bool is_leaf;
};

}; // namespace QuadTree::Msg
