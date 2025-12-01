#pragma once

#include "quadtree/QuadTreeNodeMsg.h"

namespace QuadTree {

struct QuadTreeNode {
  int8_t val;
  int x, y;
  int size;
  bool is_leaf;
  QuadTreeNode *children[4];

  QuadTreeNode(int val, int x, int y, int size, bool is_leaf);
  QuadTreeNode(Msg::QuadTreeNodeMsg &data);

  ~QuadTreeNode();

  bool contains(int qx, int qy);
  void divide();

  Msg::QuadTreeNodeMsg getInfo();
};

} // namespace QuadTree
