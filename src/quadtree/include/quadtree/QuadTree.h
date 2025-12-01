#pragma once

#include <climits>
#include <cstdint>
#include <vector>

#include "QuadTreeMsg.h"
#include "QuadTreeNode.h"
#include "quadtree/QuadTreeNodeMsg.h"

namespace QuadTree {

class QuadTree {
public:
  QuadTree(std::vector<int8_t> &grid, int height, int width, int depth);
  QuadTree(int size, int depth);
  QuadTree(Msg::QuadTreeMsg &data);
  ~QuadTree();

  int query(int x, int y);
  int getSize();
  int getHeight();
  int getWidth();
  int getDepth();

  void update(int x, int y, int val);
  void printTree();

  void deleteTree();

  Msg::QuadTreeMsg convertToMsg();

private:
  bool isValid(int x, int y);

  std::pair<bool, int>
  isHomogenous(std::vector<int8_t> &grid, int x, int y, int size);

  int query(QuadTreeNode *node, int x, int y);
  int nextPowerOf2(int n);

  void build(QuadTreeNode *node, std::vector<int8_t> &grid, int depth);
  void update(QuadTreeNode *node, int x, int y, int val, int depth);
  void printTree(QuadTreeNode *node, int depth);
  void serialize(QuadTreeNode *node, std::vector<Msg::QuadTreeNodeMsg> &out);
  QuadTreeNode *deserialize(std::vector<Msg::QuadTreeNodeMsg> &msg, int &idx);

private:
  QuadTreeNode *root_;
  int size_, depth_;
  int height_, width_;
  int max_depth_;

  static constexpr Msg::QuadTreeNodeMsg null_node_ = {INT8_MIN, 0, 0, 0, false};
};

} // namespace QuadTree
