#include "quadtree/QuadTree.h"
#include "quadtree/QuadTreeNode.h"

#include <stdexcept>

enum class Direction {
  TopLeft     = 0,
  TopRight    = 1,
  BottomLeft  = 2,
  BottomRight = 3
};

namespace QuadTree {

QuadTree::QuadTree(
    std::vector<int> &grid, int height, int width, int depth = 10
)
    : root_(nullptr), depth_(depth), height_(height), width_(width) {
  size_ = std::max(height, width);
  size_ = (size_ % 2) ? size_ : size_ + 1;
  root_ = build(grid, 0, 0, size_, 0);
}

QuadTree::QuadTree(int size, int depth = 10)
    : root_(nullptr), depth_(depth), height_(size), width_(size) {
  size_ = (size % 2) ? size : size + 1;
  root_ = new QuadTreeNode(0, 0, 0, size, true);
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

void QuadTree::update(int x, int y, int val) {
  if (!isValid(x, y))
    throw std::runtime_error("QuadTree index out of bounds");

  return update(root_, x, y, val, 0);
}

void QuadTree::printTree() { printTree(root_, 0); }

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
QuadTree::isHomogenous(std::vector<int> &grid, int x, int y, int size) {
  bool seen_free     = false;
  bool seen_obstacle = false;

  for (int i = x; i < x + size; i++) {
    for (int j = y; j < y + size; j++) {
      int cell = 0;
      if (isValid(i, j))
        cell = grid[i * width_ + j];

      if (cell == 0)
        seen_free = true;
      else
        seen_obstacle = true;

      if (seen_free && seen_obstacle)
        return {false, 0};
    }
  }

  return {true, seen_obstacle ? 100 : 0};
}

QuadTreeNode *
QuadTree::build(std::vector<int> &grid, int x, int y, int size, int depth) {
  auto homogeneity = isHomogenous(grid, x, y, size);
  if (depth >= depth_ || homogeneity.first) {
    return new QuadTreeNode(homogeneity.second, x, y, size, true);
  }

  QuadTreeNode *curr = new QuadTreeNode(0, x, y, size, false);
  int child_size     = size / 2;

  curr->children[(int)Direction::TopLeft] =
      build(grid, x, y, child_size, depth + 1);

  curr->children[(int)Direction::TopRight] =
      build(grid, x, y + child_size, child_size, depth + 1);

  curr->children[(int)Direction::BottomLeft] =
      build(grid, x + child_size, y, child_size, depth + 1);

  curr->children[(int)Direction::BottomRight] =
      build(grid, x + child_size, y + child_size, child_size, depth + 1);

  return curr;
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

void QuadTree::update(QuadTreeNode *node, int x, int y, int val, int depth) {
  if (!node)
    return;

  if (depth >= depth_ || node->size == 1) {
    node->is_leaf = true;
    node->val     = val;
    return;
  }

  if (node->is_leaf) {
    node->is_leaf = false;
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
  int v         = node->children[0]->val;
  for (int i = 1; i < 4; i++) {
    QuadTreeNode *child = node->children[i];
    if (!child || !child->is_leaf || child->val != v) {
      all_same = false;
      break;
    }
  }

  if (all_same) {
    node->is_leaf = true;
    node->val     = v;
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

}; // namespace QuadTree
