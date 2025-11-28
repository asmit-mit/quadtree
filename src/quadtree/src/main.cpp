#include "quadtree/QuadTree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class QuadTreeVizNode : public rclcpp::Node {
public:
  QuadTreeVizNode() : Node("quadtree_viz_node") {
    timer_base_ = this->create_wall_timer(
        500ms, std::bind(&QuadTreeVizNode::timerCallback, this)
    );
  }

private:
  void timerCallback() {
    std::vector<int> grid = {0, 100, 100, 100, 0, 100, 100, 100};
    int height            = 2;
    int width             = 4;

    QuadTree::QuadTree tree(grid, height, width, 10);
    RCLCPP_INFO(
        this->get_logger(), "Query (0 0): %d, Expected: 0", tree.query(0, 0)
    );
    RCLCPP_INFO(
        this->get_logger(), "Query (0 1): %d, Expected: 100", tree.query(0, 1)
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_base_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadTreeVizNode>());
  rclcpp::shutdown();
  return 0;
}
