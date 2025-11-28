#pragma once

#include <vector>

#include "QuadTreeNode.h"

namespace QuadTree {

class QuadTree {
public:
  QuadTree(std::vector<int> &grid, int height, int width, int depth);
  QuadTree(int size, int depth);
  ~QuadTree();

  int query(int x, int y);
  int getSize();
  int getHeight();
  int getWidth();

  void update(int x, int y, int val);
  void printTree();

private:
  bool isValid(int x, int y);

  std::pair<bool, int>
  isHomogenous(std::vector<int> &grid, int x, int y, int size);

  QuadTreeNode *
  build(std::vector<int> &grid, int x, int y, int size, int depth);
  int query(QuadTreeNode *node, int x, int y);

  void update(QuadTreeNode *node, int x, int y, int val, int depth);
  void printTree(QuadTreeNode *node, int depth);

private:
  QuadTreeNode *root_;
  int size_, depth_;
  int height_, width_;
};

} // namespace QuadTree
