#pragma once

#include <cstdint>
#include <vector>

#include "QuadTreeNode.h"

namespace QuadTree {

class QuadTree {
public:
  QuadTree(std::vector<int8_t> &grid, int height, int width, int depth);
  QuadTree(int size, int depth);
  ~QuadTree();

  int query(int x, int y);
  int getSize();
  int getHeight();
  int getWidth();
  int getDepth();

  void update(int x, int y, int val);
  void printTree();

  // TODO
  // serialize
  // deserialize

private:
  bool isValid(int x, int y);

  std::pair<bool, int>
  isHomogenous(std::vector<int8_t> &grid, int x, int y, int size);

  QuadTreeNode *
  build(std::vector<int8_t> &grid, int x, int y, int size, int depth);

  int query(QuadTreeNode *node, int x, int y);
  int nextPowerOf2(int n);

  void update(QuadTreeNode *node, int x, int y, int val, int depth);
  void printTree(QuadTreeNode *node, int depth);

private:
  QuadTreeNode *root_;
  int size_, depth_;
  int height_, width_;
  int max_depth_;
};

} // namespace QuadTree
