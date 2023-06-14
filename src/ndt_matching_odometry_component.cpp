#include "ndt_matching_odometry/ndt_matching_odometry_component.hpp"

namespace ndt_matching_odometry
{
    NDTMatchingOdometry::NDTMatchingOdometry(const rclcpp::NodeOptions & options) : Node("ndt_matching_odometry", options)
    {
        velodyne_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne", 10, std::bind(&NDTMatchingOdometry::velodyne_callback, this, std::placeholders::_1));
        map_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne", 10, std::bind(&NDTMatchingOdometry::slam_pointcloud_callback, this, std::placeholders::_1));

    }

    void NDTMatchingOdometry::velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
    {
        velodyne_ = *data;
    }

    void NDTMatchingOdometry::slam_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
    {
        map_ = *data;
    }

    void NDTMatchingOdometry::convert_msgtopointcloud(void)
    {
        pcl::fromROSMsg(velodyne_ ,*velodyne_point_);
        pcl::fromROSMsg(map_ ,*map_point_);
    }

    void NDTMatchingOdometry::voxcel_filter(void)
    {
        approximate_voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
        approximate_voxel_filter_.setInputCloud(velodyne_point_);
        approximate_voxel_filter_.filter (*velodyne_filtered_cloud_);
        approximate_voxel_filter_.filter (*map_filtered_cloud_);

    }

    void NDTMatchingOdometry::calc_NormalDistributionsTransform(void)
    {
        ndt_.setTransformationEpsilon(0.01);
        ndt_.setStepSize(0.1);
        ndt_.setResolution(1.0);
        ndt_.setMaximumIterations(35);
        ndt_.setInputSource(velodyne_filtered_cloud_);
        ndt_.setInputTarget(map_filtered_cloud_);
    }

    NDTMatchingOdometry::~NDTMatchingOdometry(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(ndt_matching_odometry::NDTMatchingOdometry)