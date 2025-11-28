cmake_minimum_required(VERSION 3.8)
project(quadtree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# library
# let the compiler search for headers in the include folder
include_directories(include)
 
# define a library target called my_value_converter_library
add_library(${PROJECT_NAME} 
  src/QuadTree.cpp
  src/QuadTreeNode.cpp
)
 
# this line to exports the library
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)
 
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# nodes
add_executable(quadtree_viz_node src/main.cpp)
ament_target_dependencies(quadtree_viz_node rclcpp std_msgs)
target_link_libraries(quadtree_viz_node ${PROJECT_NAME})

install(TARGETS quadtree_viz_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
