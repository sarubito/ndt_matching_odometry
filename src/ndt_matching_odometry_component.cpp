#include "ndt_matching_odometry/ndt_matching_odometry_component.hpp"

namespace ndt_matching_odometry
{
    NDTMatchingOdometry::NDTMatchingOdometry(const rclcpp::NodeOptions & options) : Node("ndt_matching_odometry", options)
    {
        velodyne_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne", 10, std::bind(&NDTMatchingOdometry::velodyne_callback, this, std::placeholders::_1));
        slam_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne", 10, std::bind(&NDTMatchingOdometry::slam_pointcloud_callback, this, std::placeholders::_1));

    }

    void NDTMatchingOdometry::velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
    {
        velodyne_ = *data;
    }

    void NDTMatchingOdometry::slam_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
    {
        map_ = *data;
    }

    NDTMatchingOdometry::~NDTMatchingOdometry(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(ndt_matching_odometry::NDTMatchingOdometry)