#ifndef NDT_MATCHING_ODOMETRY_COMPONENT_HPP_
#define NDT_MATCHING_ODOMETRY_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NDT_MATCHING_ODOMETRY_COMPONENT_EXPORT __attribute__((dllexport))
#define NDT_MATCHING_ODOMETRY_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NDT_MATCHING_ODOMETRY_COMPONENT_EXPORT __declspec(dllexport)
#define NDT_MATCHING_ODOMETRY_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NDT_MATCHING_ODOMETRY_COMPONENT_BUILDING_DLL
#define NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC \
  NDT_MATCHING_ODOMETRY_COMPONENT_EXPORT
#else
#define NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC \
  NDT_MATCHING_ODOMETRY_COMPONENT_IMPORT
#endif
#define NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC_TYPE \
  NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC
#define NDT_MATCHING_ODOMETRY_COMPONENT_LOCAL
#else
#define NDT_MATCHING_ODOMETRY_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define NDT_MATCHING_ODOMETRY_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define NDT_MATCHING_ODOMETRY_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC
#define NDT_MATCHING_ODOMETRY_COMPONENT_LOCAL
#endif
#define NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std::chrono_literals;

namespace ndt_matching_odometry
{
    class NDTMatchingOdometry : public rclcpp::Node
    {
        public:
            NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC
            explicit NDTMatchingOdometry(const rclcpp::NodeOptions & options);
            virtual ~NDTMatchingOdometry(void);

        private:
            void velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data);
            void slam_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data);
            void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr data);
            void convert_msgtopointcloud(void);
            void calc_NormalDistributionsTransform(void);
            void voxcel_filter(void);
            void initialpose(void);
            void RangeFilter(void);
            void process(void);
            Eigen::Quaternionf convertMsgtoEigen(void);
            geometry_msgs::msg::Quaternion convertEigentoMsg(Eigen::Quaternionf q_eigen);

            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_subscription_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

            sensor_msgs::msg::PointCloud2 velodyne_;
            sensor_msgs::msg::PointCloud2 map_;
            nav_msgs::msg::Odometry robot_odometry_;
            geometry_msgs::msg::PoseStamped ndt_pose_;

            pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_point_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_point_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_range_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::PointCloud<pcl::PointXYZ>::Ptr velodyne_filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;

            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            Eigen::Translation3f translation;
            Eigen::AngleAxisf rotation;
            Eigen::Matrix4f guess;

    };
}


#endif