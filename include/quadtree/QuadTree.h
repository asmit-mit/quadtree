#pragma once

#include <climits>
#include <cstdint>
#include <utility>
#include <vector>

#include "QuadTreeNode.h"
#include "quadtree/msg/quad_tree.hpp"

namespace quadtree {

class QuadTree {
public:
  QuadTree(std::vector<int8_t> &grid, int height, int width, int depth = 10);
  QuadTree(int size, int depth = 10);
  QuadTree(quadtree::msg::QuadTree &data);
  ~QuadTree();

  QuadTree(const QuadTree &) = delete;
  QuadTree &operator=(const QuadTree &) = delete;

  int query(int x, int y);
  int getSize();
  int getHeight();
  int getWidth();
  int getDepth();

  void update(int x, int y, int val);
  void printTree();

  void deleteTree();

  quadtree::msg::QuadTree toROSMsg();

private:
  bool isValid(int x, int y);

  bool outside(const QuadTreeNode *n) const;
  void divideClipped(QuadTreeNode *node);

  int obstacleCount(const std::vector<int> &prefix, int x, int y, int xe,
                    int ye) const;

  std::pair<bool, int> isHomogenous(const std::vector<int> &prefix, int x,
                                    int y, int size) const;

  int query(QuadTreeNode *node, int x, int y);
  int nextPowerOf2(int n);

  void build(QuadTreeNode *node, const std::vector<int> &prefix, int depth);
  void update(QuadTreeNode *node, int x, int y, int val, int depth);
  void printTree(QuadTreeNode *node, int depth);
  void serialize(QuadTreeNode *node,
                 std::vector<quadtree::msg::QuadTreeNode> &out);
  QuadTreeNode *deserialize(std::vector<quadtree::msg::QuadTreeNode> &msg,
                            int &idx);

private:
  QuadTreeNode *root_;
  int size_, depth_;
  int height_, width_;
  int max_depth_;

  static constexpr int NULL_MARKER = INT8_MIN;
  quadtree::msg::QuadTreeNode null_node_;
};

} // namespace quadtree
