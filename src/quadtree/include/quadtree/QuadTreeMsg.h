#pragma once

#include "QuadTreeNodeMsg.h"

#include <vector>

namespace QuadTree::Msg {

struct QuadTreeMsg {
  int height, width;
  int depth, size;
  int max_depth;

  std::vector<QuadTree::Msg::QuadTreeNodeMsg> nodes;
};

}; // namespace QuadTree::Msg
