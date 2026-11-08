#include "quadtree/QuadTree.h"
#include "quadtree/QuadTreeNode.h"

#include <algorithm>
#include <cstdio>
#include <stdexcept>

namespace quadtree {

QuadTree::QuadTree(std::vector<int8_t> &grid, int height, int width, int depth)
    : root_(nullptr), size_(0), depth_(0), height_(height), width_(width),
      max_depth_(depth) {
  if (height <= 0 || width <= 0)
    throw std::invalid_argument("QuadTree dimensions must be positive");
  if ((long long)height * width > INT_MAX)
    throw std::invalid_argument("QuadTree grid too large");
  if (grid.size() < (size_t)height * (size_t)width)
    throw std::invalid_argument("QuadTree grid smaller than height * width");

  null_node_ = quadtree::msg::QuadTreeNode();
  null_node_.val = NULL_MARKER;

  size_ = nextPowerOf2(std::max(height, width));
  root_ = new QuadTreeNode(0, 0, 0, size_, true);

  const int stride = width + 1;
  std::vector<int> prefix((size_t)(height + 1) * stride, 0);
  for (int i = 0; i < height; i++) {
    int row_sum = 0;
    for (int j = 0; j < width; j++) {
      int8_t cell = grid[(size_t)i * width + j];
      row_sum += (cell != 0 && cell != -1) ? 1 : 0;
      prefix[(size_t)(i + 1) * stride + (j + 1)] =
          prefix[(size_t)i * stride + (j + 1)] + row_sum;
    }
  }

  build(root_, prefix, 0);
}

QuadTree::QuadTree(int size, int depth)
    : root_(nullptr), size_(0), depth_(0), height_(size), width_(size),
      max_depth_(depth) {
  if (size <= 0)
    throw std::invalid_argument("QuadTree size must be positive");

  null_node_ = quadtree::msg::QuadTreeNode();
  null_node_.val = NULL_MARKER;

  size_ = nextPowerOf2(size);
  root_ = new QuadTreeNode(0, 0, 0, size_, true);
}

QuadTree::QuadTree(quadtree::msg::QuadTree &data)
    : root_(nullptr), size_(data.info.size), depth_(data.info.depth),
      height_(data.info.height), width_(data.info.width),
      max_depth_(data.info.max_depth) {
  null_node_ = quadtree::msg::QuadTreeNode();
  null_node_.val = NULL_MARKER;

  int idx = 0;
  root_ = deserialize(data.nodes, idx);
}

QuadTree::~QuadTree() {
  delete root_;
  root_ = nullptr;
}

int QuadTree::query(int x, int y) {
  if (!isValid(x, y))
    throw std::runtime_error("QuadTree index out of bounds");

  return query(root_, x, y);
}
int QuadTree::getSize() { return size_; }
int QuadTree::getHeight() { return height_; }
int QuadTree::getWidth() { return width_; }
int QuadTree::getDepth() { return depth_; }

void QuadTree::update(int x, int y, int val) {
  if (!isValid(x, y))
    throw std::runtime_error("QuadTree index out of bounds");

  update(root_, x, y, val, 0);
}

void QuadTree::printTree() { printTree(root_, 0); }

void QuadTree::deleteTree() {
  delete root_;
  root_ = nullptr;
}

quadtree::msg::QuadTree QuadTree::toROSMsg() {
  quadtree::msg::QuadTree out;
  out.info.height = height_;
  out.info.width = width_;
  out.info.depth = depth_;
  out.info.size = size_;
  out.info.max_depth = max_depth_;

  serialize(root_, out.nodes);

  return out;
}

bool QuadTree::isValid(int x, int y) {
  return (x >= 0 && y >= 0 && x < height_ && y < width_);
}

bool QuadTree::outside(const QuadTreeNode *n) const {
  return n->x >= height_ || n->y >= width_;
}

void QuadTree::divideClipped(QuadTreeNode *node) {
  node->divide();
  for (int i = 0; i < 4; i++) {
    if (node->children[i] && outside(node->children[i])) {
      delete node->children[i];
      node->children[i] = nullptr;
    }
  }
}

int QuadTree::obstacleCount(const std::vector<int> &prefix, int x, int y,
                            int xe, int ye) const {
  const size_t stride = (size_t)width_ + 1;
  return prefix[xe * stride + ye] - prefix[x * stride + ye] -
         prefix[xe * stride + y] + prefix[x * stride + y];
}

std::pair<bool, int> QuadTree::isHomogenous(const std::vector<int> &prefix,
                                            int x, int y, int size) const {
  int xe = std::min(x + size, height_);
  int ye = std::min(y + size, width_);

  int cnt = obstacleCount(prefix, x, y, xe, ye);
  int area = (xe - x) * (ye - y);

  if (cnt == 0)
    return {true, 0};
  if (cnt == area)
    return {true, 100};
  return {false, 100};
}

void QuadTree::build(QuadTreeNode *node, const std::vector<int> &prefix,
                     int depth) {
  auto homogeneity = isHomogenous(prefix, node->x, node->y, node->size);
  if (depth >= max_depth_ || homogeneity.first) {
    depth_ = std::max(depth, depth_);
    node->val = homogeneity.second;
    return;
  }

  divideClipped(node);
  for (int i = 0; i < 4; i++) {
    if (node->children[i])
      build(node->children[i], prefix, depth + 1);
  }
}

int QuadTree::query(QuadTreeNode *node, int x, int y) {
  if (!node)
    return -1;

  if (node->is_leaf)
    return node->val;

  for (int i = 0; i < 4; i++) {
    QuadTreeNode *child = node->children[i];
    if (child && child->contains(x, y)) {
      return query(child, x, y);
    }
  }

  return -1;
}

int QuadTree::nextPowerOf2(int n) {
  int p = 1;
  while (p < n)
    p <<= 1;
  return p;
}

void QuadTree::update(QuadTreeNode *node, int x, int y, int val, int depth) {
  if (!node)
    return;

  if (node->is_leaf && node->val == val)
    return;

  if (depth >= max_depth_ || node->size == 1) {
    depth_ = std::max(depth_, depth);
    node->is_leaf = true;
    node->val = val;
    return;
  }

  if (node->is_leaf)
    divideClipped(node);

  for (int i = 0; i < 4; i++) {
    QuadTreeNode *child = node->children[i];
    if (child && child->contains(x, y)) {
      update(child, x, y, val, depth + 1);
      break;
    }
  }

  bool all_same = true, any = false;
  int v = 0;
  for (int i = 0; i < 4; i++) {
    QuadTreeNode *c = node->children[i];
    if (!c)
      continue;
    if (!c->is_leaf) {
      all_same = false;
      break;
    }
    if (!any) {
      v = c->val;
      any = true;
    } else if (c->val != v) {
      all_same = false;
      break;
    }
  }

  if (all_same && any) {
    node->is_leaf = true;
    node->val = v;
    for (int i = 0; i < 4; i++) {
      delete node->children[i];
      node->children[i] = nullptr;
    }
  }
}

void QuadTree::printTree(QuadTreeNode *node, int depth) {
  if (!node) {
    for (int i = 0; i < depth; i++)
      printf("  ");
    printf("null\n");
    return;
  }

  for (int i = 0; i < depth; i++)
    printf("  ");
  printf("Node value: %d\n", node->val);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < depth; j++)
      printf("  ");
    printf("Child %d:\n", i);
    printTree(node->children[i], depth + 1);
  }
}

void QuadTree::serialize(QuadTreeNode *node,
                         std::vector<quadtree::msg::QuadTreeNode> &out) {
  if (!node) {
    out.push_back(null_node_);
    return;
  }

  out.push_back(node->getInfo());
  for (int i = 0; i < 4; i++)
    serialize(node->children[i], out);
}

QuadTreeNode *
QuadTree::deserialize(std::vector<quadtree::msg::QuadTreeNode> &data,
                      int &idx) {
  if (idx >= (int)data.size())
    return nullptr;

  auto val = data[idx++];
  if (val.val == NULL_MARKER)
    return nullptr;

  QuadTreeNode *node = new QuadTreeNode(val);
  for (int i = 0; i < 4; i++)
    node->children[i] = deserialize(data, idx);

  return node;
}

} // namespace quadtree
