#include "nav_msgs/msg/occupancy_grid.hpp"
#include "quadtree/QuadTree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;

class QuadTreeVizNode : public rclcpp::Node {
public:
  QuadTreeVizNode() : Node("quadtree_viz_node") {
    timer_base_ = this->create_wall_timer(
        500ms, std::bind(&QuadTreeVizNode::timerCallback, this)
    );

    auto qos = rclcpp::QoS(10).reliable().transient_local();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&QuadTreeVizNode::mapCallback, this, _1)
    );
  }

private:
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int height               = msg->info.height;
    int width                = msg->info.width;
    std::vector<int8_t> data = msg->data;

    RCLCPP_INFO(
        this->get_logger(), "Height and width from msg: %d %d", height, width
    );
  }

  void timerCallback() {}

private:
  rclcpp::TimerBase::SharedPtr timer_base_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadTreeVizNode>());
  rclcpp::shutdown();
  return 0;
}
