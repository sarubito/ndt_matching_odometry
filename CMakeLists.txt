cmake_minimum_required(VERSION 3.8)
project(ndt_matching_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include 
                    ${PCL_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(ndt_matching_odometry_component SHARED src/ndt_matching_odometry_component.cpp)
rclcpp_components_register_nodes(ndt_matching_odometry_component "ndt_matching_odometry::NDTMatchingOdometry")
ament_auto_add_executable(ndt_matching_odometry_node
  src/ndt_matching_odometry_component.cpp
  src/ndt_matching_odometry_node.cpp
)

target_link_libraries(ndt_matching_odometry_node ${PCL_LIBRARIES})

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

ament_auto_package()
