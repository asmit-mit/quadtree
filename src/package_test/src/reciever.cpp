#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quadtree/QuadTree.h"
#include "quadtree/msg/quad_tree.hpp"

#include <chrono>
#include <memory>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class Reciever : public rclcpp::Node {
public:
  Reciever() : Node("reciever") {
    auto qos = rclcpp::QoS(10).reliable().transient_local();

    tree_sub_ = this->create_subscription<quadtree::msg::QuadTree>(
        "/tree", qos, std::bind(&Reciever::treeCallback, this, _1)
    );

    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/new_map", qos);
  }

private:
  void treeCallback(const quadtree::msg::QuadTree::SharedPtr msg) {
    QuadTree::QuadTree tree(*msg);
    RCLCPP_INFO(
        this->get_logger(), "Built tree! of size %d, depth %d and %zu nodes",
        tree.getSize(), tree.getDepth(), msg->nodes.size()
    );

    auto new_msg = nav_msgs::msg::OccupancyGrid();

    new_msg.header.stamp    = this->get_clock()->now();
    new_msg.header.frame_id = "map";

    new_msg.info.map_load_time = this->get_clock()->now();
    new_msg.info.resolution    = 0.13;
    new_msg.info.width         = tree.getWidth();
    new_msg.info.height        = tree.getHeight();

    new_msg.info.origin.position.x    = 0.0;
    new_msg.info.origin.position.y    = 0.0;
    new_msg.info.origin.position.z    = 0.0;
    new_msg.info.origin.orientation.w = 1.0;

    new_msg.data.resize(new_msg.info.width * new_msg.info.height);

    for (int i = 0; i < tree.getHeight(); i++) {
      for (int y = 0; y < tree.getWidth(); y++) {
        int idx           = i * new_msg.info.width + y;
        new_msg.data[idx] = static_cast<int8_t>(tree.query(i, y));
      }
    }

    map_pub_->publish(new_msg);
  }

  rclcpp::Subscription<quadtree::msg::QuadTree>::SharedPtr tree_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Reciever>());
  rclcpp::shutdown();
  return 0;
}
