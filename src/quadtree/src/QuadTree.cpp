#include "quadtree/QuadTree.h"
#include "quadtree/QuadTreeMsg.h"
#include "quadtree/QuadTreeNode.h"
#include "quadtree/QuadTreeNodeMsg.h"

#include <stdexcept>

namespace QuadTree {

QuadTree::QuadTree(
    std::vector<int8_t> &grid, int height, int width, int depth = 10
)
    : root_(nullptr), depth_(0), height_(height), width_(width),
      max_depth_(depth) {
  size_ = std::max(height, width);
  size_ = nextPowerOf2(size_);
  root_ = new QuadTreeNode(0, 0, 0, size_, true);
  build(root_, grid, 0);
}

QuadTree::QuadTree(int size, int depth = 10)
    : root_(nullptr), depth_(0), height_(size), width_(size),
      max_depth_(depth) {
  size_ = nextPowerOf2(size);
  root_ = new QuadTreeNode(0, 0, 0, size, true);
}

QuadTree::QuadTree(Msg::QuadTreeMsg &data)
    : root_(nullptr), size_(data.size), depth_(data.depth),
      height_(data.height), width_(data.width), max_depth_(data.max_depth) {
  int idx = 0;
  root_   = deserialize(data.nodes, idx);
}

QuadTree::~QuadTree() { delete root_; }

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
void QuadTree::deleteTree() { delete root_; }

Msg::QuadTreeMsg QuadTree::convertToMsg() {
  Msg::QuadTreeMsg out;
  out.height    = height_;
  out.width     = width_;
  out.depth     = depth_;
  out.size      = size_;
  out.max_depth = max_depth_;

  serialize(root_, out.nodes);

  return out;
}

bool QuadTree::isValid(int x, int y) {
  return (x >= 0 && y >= 0 && x < height_ && y < width_);
}

// std::pair<bool, int>
// QuadTree::isHomogenous(std::vector<int> &grid, int x, int y, int size) {
//   int val = isValid(x, y) ? grid[x * width_ + y] : 0;
//
//   for (int i = x; i < x + size; i++) {
//     for (int j = y; j < y + size; j++) {
//       if (!isValid(i, j)) {
//         if (val != 0)
//           return {false, val};
//         continue;
//       }
//
//       int idx = i * width_ + j;
//       if (grid[idx] != val)
//         return {false, val};
//     }
//   }
//
//   return {true, val};
// }

std::pair<bool, int>
QuadTree::isHomogenous(std::vector<int8_t> &grid, int x, int y, int size) {
  bool seen_free     = false;
  bool seen_obstacle = false;

  for (int i = x; i < x + size; i++) {
    for (int j = y; j < y + size; j++) {
      int cell = 0;
      if (isValid(i, j)) {
        cell = grid[i * width_ + j];
        cell = (cell == -1) ? 0 : cell;
      }

      if (cell == 0)
        seen_free = true;
      else
        seen_obstacle = true;

      if (seen_free && seen_obstacle)
        return {false, 100};
    }
  }

  return {true, seen_obstacle ? 100 : 0};
}

void QuadTree::build(QuadTreeNode *node, std::vector<int8_t> &grid, int depth) {
  auto homogeneity = isHomogenous(grid, node->x, node->y, node->size);
  if (depth >= max_depth_ || homogeneity.first) {
    depth_    = std::max(depth, depth_);
    node->val = homogeneity.second;
    return;
  }

  node->divide();
  for (int i = 0; i < 4; i++) {
    build(node->children[i], grid, depth + 1);
  }
}

int QuadTree::query(QuadTreeNode *node, int x, int y) {
  if (!node)
    return -1;

  if (node->is_leaf) {
    return node->val;
  }

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

  if (depth >= max_depth_ || node->size == 1) {
    depth_        = std::max(depth_, depth);
    node->is_leaf = true;
    node->val     = val;
    return;
  }

  if (node->is_leaf) {
    node->divide();
  }

  for (int i = 0; i < 4; i++) {
    QuadTreeNode *child = node->children[i];
    if (child && child->contains(x, y)) {
      update(child, x, y, val, depth + 1);
      break;
    }
  }

  bool all_same = true;

  if (node->children[0]) {
    int v = node->children[0]->val;
    for (int i = 0; i < 4; i++) {
      QuadTreeNode *child = node->children[i];
      if (!(child && child->is_leaf && child->val == v)) {
        all_same = false;
        break;
      }
    }
  } else {
    all_same = false;
  }

  if (all_same) {
    node->is_leaf = true;
    node->val     = node->children[0]->val;
    for (int i = 0; i < 4; i++) {
      delete node->children[i];
      node->children[i] = nullptr;
    }
  }
}

void QuadTree::printTree(QuadTreeNode *node, int depth = 0) {
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

void QuadTree::serialize(
    QuadTreeNode *node, std::vector<Msg::QuadTreeNodeMsg> &out
) {
  if (!node) {
    out.push_back(null_node_);
    return;
  }

  out.push_back(node->getInfo());
  for (int i = 0; i < 4; i++) {
    serialize(node->children[i], out);
  }
}

QuadTreeNode *
QuadTree::deserialize(std::vector<Msg::QuadTreeNodeMsg> &data, int &idx) {
  if (idx >= (int)data.size()) {
    return nullptr;
  }

  auto val = data[idx++];
  if (val.val == null_node_.val) {
    return nullptr;
  }

  QuadTreeNode *node = new QuadTreeNode(val);
  for (int i = 0; i < 4; i++) {
    node->children[i] = deserialize(data, idx);
  }

  return node;
}

}; // namespace QuadTree
