#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#include "quadtree/QuadTree.h"
#include "quadtree/msg/quad_tree.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;

class Maker : public rclcpp::Node {
public:
  Maker() : Node("maker") {
    auto qos = rclcpp::QoS(10).reliable().transient_local();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos, std::bind(&Maker::mapCallback, this, _1)
    );

    tree_pub_ = this->create_publisher<quadtree::msg::QuadTree>("/tree", qos);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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
    RCLCPP_INFO(this->get_logger(), "Query (0, 0): %d", tree.query(0, 0));

    auto data = tree.toROSMsg();

    tree_pub_->publish(data);
    RCLCPP_INFO(this->get_logger(), "Published at topic tree");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<quadtree::msg::QuadTree>::SharedPtr tree_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Maker>());
  rclcpp::shutdown();
  return 0;
}
