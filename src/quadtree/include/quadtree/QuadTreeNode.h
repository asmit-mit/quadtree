#pragma once

namespace QuadTree {

struct QuadTreeNode {
  int val;
  int x, y;
  int size;
  bool is_leaf;
  QuadTreeNode *children[4];

  QuadTreeNode(int val, int x, int y, int size, bool is_leaf);
  ~QuadTreeNode();

  bool contains(int qx, int qy);
  void divide();
};

} // namespace QuadTree
