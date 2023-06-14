#include "ndt_matching_odometry/ndt_matching_odometry_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto ndt_matching_odometry_component = std::make_shared<ndt_matching_odometry::NDTMatchingOdometry>(rclcpp::NodeOptions());
    exec.add_node(ndt_matching_odometry_component);
    exec.spin();
    rclcpp::shutdown();
}