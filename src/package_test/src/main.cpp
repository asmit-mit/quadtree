#include "nav_msgs/msg/occupancy_grid.hpp"
#include "quadtree/QuadTree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;

class QuadTreeVizNode : public rclcpp::Node {
public:
  QuadTreeVizNode() : Node("quadtree_viz_node") {
    // timer_base_ = this->create_wall_timer(
    //     500ms, std::bind(&QuadTreeVizNode::timerCallback, this)
    // );

    auto qos = rclcpp::QoS(10).reliable().transient_local();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&QuadTreeVizNode::mapCallback, this, _1)
    );

    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/new_map", qos);
  }

private:
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int height = msg->info.height;
    int width  = msg->info.width;

    RCLCPP_INFO(
        this->get_logger(), "Height and width from msg: %d %d", height, width
    );

    QuadTree::QuadTree tree(msg->data, height, width, 8);
    RCLCPP_INFO(
        this->get_logger(), "QuadTree built! size: %d and depth: %d",
        tree.getSize(), tree.getDepth()
    );

    nav_msgs::msg::OccupancyGrid new_msg;
    new_msg.header = msg->header;
    new_msg.info   = msg->info;

    new_msg.data.resize(height * width);
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        int idx           = i * width + j;
        new_msg.data[idx] = tree.query(i, j);
      }
    }

    map_pub_->publish(new_msg);
  }

  // void timerCallback() {}

private:
  // rclcpp::TimerBase::SharedPtr timer_base_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadTreeVizNode>());
  rclcpp::shutdown();
  return 0;
}
