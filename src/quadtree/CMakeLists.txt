cmake_minimum_required(VERSION 3.8)
project(quadtree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuadTreeNode.msg"
  "msg/QuadTree.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# library
include_directories(include)
 
# define a library target called my_value_converter_library
add_library(${PROJECT_NAME}_core
  src/QuadTree.cpp
  src/QuadTreeNode.cpp
)

rosidl_target_interfaces(${PROJECT_NAME}_core ${PROJECT_NAME} "rosidl_typesupport_cpp")

# this line to exports the library
ament_export_targets(export_${PROJECT_NAME}_core HAS_LIBRARY_TARGET)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)
 
install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
