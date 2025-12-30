# Quadtree for ROS 2 Humble

## Overview

This is a hobby project I built to see how a quadtree will perform compared to a standard occupancy grid.
This is a drop-in replacement for `nav_msgs::msg::OccupancyGrid`, designed to be simple to integrate, with no dependencies.

## Features

- Can be constructed directly from a `nav_msgs::msg::OccupancyGrid`.
- Also provides freedom to create and update tree manually.
- Supports ROS message serialization.

## Installation

1. Clone the repo

```bash
cd <your-workspace-src>
git clone https://github.com/asmit-mit/quadtree.git
```

2. Build the package

```bash
colcon build --packages-select quadtree
```

## Integration

To the package who wants to use quadtrees

1. Add the following in your `CMakeLists.txt`

```cmake
find_package(quadtree REQUIRED)

# for the node who wants quadtrees
ament_target_dependencies(<your-node> <othre-dependencies> quadtree)
```

2. Add dependency in your `package.xml`

```xml
<depend>quadtree</depend>
```

## Using the API

### Generating the Tree

The quadtree assumes the size of the nearest power of 2 of max(height, width) of the map. This is done for smoother sub-division.
Though you can't access the region outside the (height and width) of the map, it'll give you runtime error.
I choose this to increase consistency with the occupancy grid i.e if you can't access (x,y) in the occupancy grid then you shouldn't be able to access it in the quadtree either.

1. Directly from a `nav_msgs::msg::OccupancyGrid`:

This constructor can be used to create quadtree from any vector<int8_t>!

```cpp
// General constructor
// Quadtree(vector<int8_t>& &grid, int height, int width, int max_depth);

// Example using a callback function
void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int height = msg->info.height;
    int width  = msg->info.width;

    RCLCPP_INFO(
        this->get_logger(), "Height and width from map: %d %d", height, width
    );

    int max_depth = 10;
    quadtree::QuadTree tree(msg->data, height, width, max_depth);
    RCLCPP_INFO(
        this->get_logger(), "QuadTree built! size: %d and depth: %d",
        tree.getSize(), tree.getDepth()
    );
}
```

2. Initializing an empty tree of any size:

```cpp
// The constructor creates a map of same height and width being equal to size
// Quadtree(int size, int max_depth);

int map_size = 1000;
int max_depth = 10;
quadtree::QuadTree tree(map_size, max_depth);
```

3. Using a quadtree msg (useful for node communication):

```cpp
// Constructor
// QuadTree(quadtree::msg::QuadTree &data);


void treeCallback(const quadtree::msg::QuadTree::SharedPtr msg) {
    quadtree::QuadTree tree(*msg);
}
```

### Querying and Updating the tree

- Get value of (x,y) from the map

```cpp
tree.query(x, y);
```

- Update value at (x,y) in the map

```cpp
tree.update(x, y, val);
```

### Publishing the Tree

- Message definition for `quadtree::msg::QuadTree`

```
std_msgs/Header header

QuadTreeMetaData info

QuadTreeNode[] nodes
```

- QuadTreeMetaData

```
uint32 height
uint32 width
uint32 size
uint32 depth
uint32 max_depth

geometry_msgs/Pose origin
```

- Example

```cpp
// publisher object
rclcpp::Publisher<quadtree::msg::QuadTree>::SharedPtr tree_pub_;
tree_pub_ = this->create_publisher<quadtree::msg::QuadTree>("/tree", qos);

// suppose we have the follwing tree already built
int map_size = 1000, max_depth = 10
quadtree::QuadTree tree(map_size, max_depth);

// get the quadtree msg
auto msg = tree.toROSMsg();
msg->header.frame_id = "map";
msg->header.stamp = this->now();

// metadata
msg->info.width     = map_size;
msg->info.height    = map_size;
msg->info.size      = map_size;
msg->info.depth     = tree.getDepth();
msg->info.max_depth = max_depth;

// origin pose (you can put your own pose)
msg->info.origin.position.x = 0.0;
msg->info.origin.position.y = 0.0;
msg->info.origin.position.z = 0.0;

msg->info.origin.orientation.x = 0.0;
msg->info.origin.orientation.y = 0.0;
msg->info.origin.orientation.z = 0.0;
msg->info.origin.orientation.w = 1.0;

tree_pub_->publish(msg);
```

## TODO (maybe)

- Apply TF2 transform directly to the quadtree (so it can change dynamically change pose metadata)
- Add a way to visualize quadtree in rviz

## Why this exists

Because:

- OccupancyGrid blows up for large maps
- Quadtrees handle sparse data better
- I wanted to test performance

If this helps you, awesome.
If it breaks, you get to keep both pieces.
