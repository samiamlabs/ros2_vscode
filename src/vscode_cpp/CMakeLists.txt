cmake_minimum_required(VERSION 3.8)
project(vscode_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include directories
include_directories(${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
ament_auto_find_build_dependencies()

# library
ament_auto_add_library(incrementer_node_lib SHARED src/incrementer_node.cpp)
ament_target_dependencies(incrementer_node_lib rclcpp std_msgs)

# executable
add_executable(incrementer_node src/incrementer_node.cpp)
ament_target_dependencies(incrementer_node rclcpp std_msgs)

# Install cpp executables
install(TARGETS
  incrementer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_incrementer_node test/test_incrementer_node.cpp)
  target_link_libraries(test_incrementer_node incrementer_node_lib)

endif()

ament_package()