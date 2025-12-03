#pragma once

#include "quadtree/msg/quad_tree_node.hpp"

namespace QuadTree {

struct QuadTreeNode {
  int8_t val;
  int x, y;
  int size;
  bool is_leaf;
  QuadTreeNode *children[4];

  QuadTreeNode(int val, int x, int y, int size, bool is_leaf);
  QuadTreeNode(quadtree::msg::QuadTreeNode &data);

  ~QuadTreeNode();

  bool contains(int qx, int qy);
  void divide();

  quadtree::msg::QuadTreeNode getInfo();
};

} // namespace QuadTree
