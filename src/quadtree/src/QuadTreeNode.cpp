#include "quadtree/QuadTreeNode.h"

namespace QuadTree {

QuadTreeNode::QuadTreeNode(int val, int x, int y, int size, bool is_leaf)
    : val(val), x(x), y(y), size(size), is_leaf(is_leaf) {
  for (int i = 0; i < 4; i++) {
    children[i] = nullptr;
  }
}

QuadTreeNode::~QuadTreeNode() {
  for (int i = 0; i < 4; i++) {
    delete children[i];
    children[i] = nullptr;
  }
}

bool QuadTreeNode::contains(int qx, int qy) {
  return (qx >= x && qy >= y && qx < x + size && qy < y + size);
}

void QuadTreeNode::divide() {
  int child_size = size / 2;

  children[0] = new QuadTreeNode(val, x, y, child_size, true);
  children[1] = new QuadTreeNode(val, x, y + child_size, child_size, true);
  children[2] = new QuadTreeNode(val, x + child_size, y, child_size, true);
  children[3] =
      new QuadTreeNode(val, x + child_size, y + child_size, child_size, true);
}

} // namespace QuadTree
