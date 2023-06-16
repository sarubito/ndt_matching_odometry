#include "ndt_matching_odometry/ndt_matching_odometry_component.hpp"

namespace ndt_matching_odometry
{
    NDTMatchingOdometry::NDTMatchingOdometry(const rclcpp::NodeOptions & options) : Node("ndt_matching_odometry", options)
    {
        velodyne_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10, std::bind(&NDTMatchingOdometry::velodyne_callback, this, std::placeholders::_1));
        map_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/map_pointcloud", 10, std::bind(&NDTMatchingOdometry::slam_pointcloud_callback, this, std::placeholders::_1));
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&NDTMatchingOdometry::odometry_callback, this, std::placeholders::_1));
        ndt_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ndt_pose", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&NDTMatchingOdometry::process, this));
    }

    void NDTMatchingOdometry::velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
    {
        velodyne_ = *data;
    }

    void NDTMatchingOdometry::slam_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
    {
        map_ = *data;
    }

    void NDTMatchingOdometry::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr data)
    {
        robot_odometry_ = *data;
    }

    void NDTMatchingOdometry::convert_msgtopointcloud(void)
    {
        pcl::fromROSMsg(velodyne_ ,*velodyne_point_);
        pcl::fromROSMsg(map_ ,*map_point_);
    }

    void NDTMatchingOdometry::RangeFilter(void)
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(velodyne_point_);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-100.0, 100.0);
        pass.filter(*velodyne_range_cloud_);
        pass.setInputCloud(velodyne_range_cloud_);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-100.0, 100.0);
        pass.filter(*velodyne_range_cloud_);
    }

    void NDTMatchingOdometry::voxcel_filter(void)
    {
        voxel_filter_.setInputCloud(velodyne_range_cloud_);
        voxel_filter_.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter_.filter(*velodyne_filtered_cloud_);
        voxel_filter_.setInputCloud(map_point_);
        voxel_filter_.setLeafSize(0.5, 0.5, 0.5);
        voxel_filter_.filter(*map_filtered_cloud_);

        RCLCPP_INFO(this->get_logger(), "velodyne point_width : %d", velodyne_range_cloud_->width);
        RCLCPP_INFO(this->get_logger(), "velodyne point_height : %d", velodyne_range_cloud_->height);
        RCLCPP_INFO(this->get_logger(), "velodyne filtered_width : %d", velodyne_filtered_cloud_->width);
        RCLCPP_INFO(this->get_logger(), "velodyne filtered_height : %d", velodyne_filtered_cloud_->height);

    }

    void NDTMatchingOdometry::calc_NormalDistributionsTransform(void)
    {
        ndt_.setTransformationEpsilon(1.0e-8);
        ndt_.setStepSize(0.1);
        ndt_.setResolution(1.0);
        ndt_.setMaximumIterations(100);
        ndt_.setInputSource(velodyne_filtered_cloud_);
        ndt_.setInputTarget(map_filtered_cloud_);
        translation = {robot_odometry_.pose.pose.position.x, robot_odometry_.pose.pose.position.y, robot_odometry_.pose.pose.position.z};
        rotation = (convertMsgtoEigen());
        guess = (translation*rotation).matrix();
        ndt_.align(*output_cloud_, guess);

        Eigen::Matrix4f T = ndt_.getFinalTransformation();
        Eigen::Matrix3f R = T.block(0, 0, 3, 3);
        Eigen::Quaternionf q_rot(R);
        q_rot.normalize();

        ndt_pose_.pose.position.x = T(0,3);
        ndt_pose_.pose.position.y = T(1,3);
        ndt_pose_.pose.position.y = T(2,3);
        ndt_pose_.pose.orientation = convertEigentoMsg(q_rot);
    }

    Eigen::Quaternionf NDTMatchingOdometry::convertMsgtoEigen(void)
    {
        Eigen::Quaternionf eigen_quat(robot_odometry_.pose.pose.orientation.w, robot_odometry_.pose.pose.orientation.x, robot_odometry_.pose.pose.orientation.y, robot_odometry_.pose.pose.orientation.z);
        eigen_quat.normalize();
        return eigen_quat;
    }

    geometry_msgs::msg::Quaternion NDTMatchingOdometry::convertEigentoMsg(Eigen::Quaternionf q_eigen)
    {
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = (double)q_eigen.x();
        q_msg.y = (double)q_eigen.y();
        q_msg.z = (double)q_eigen.z();
        q_msg.w = (double)q_eigen.w();
	    return q_msg;
    }

    void NDTMatchingOdometry::process(void)
    {
        convert_msgtopointcloud();
        RangeFilter();
        voxcel_filter();
        calc_NormalDistributionsTransform();
        ndt_pose_.header.frame_id = "ndt";
        ndt_pose_.header.stamp = this->get_clock()->now();
        ndt_pose_publisher_->publish(ndt_pose_);
    }

    NDTMatchingOdometry::~NDTMatchingOdometry(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(ndt_matching_odometry::NDTMatchingOdometry)