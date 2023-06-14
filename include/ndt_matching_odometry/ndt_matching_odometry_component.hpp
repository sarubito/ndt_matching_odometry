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
#include "rclcpp_components/register_node_macro.hpp"


namespace ndt_matching_odometry
{
    class NDTMatchingOdometry : public rclcpp::Node
    {
        public:
            NDT_MATCHING_ODOMETRY_COMPONENT_PUBLIC
            explicit NDTMatchingOdometry(const rclcpp::NodeOptions & options);
            virtual ~NDTMatchingOdometry(void);

        private:
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_pointcloud_;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr slam_pointcloud_;

            void velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data);
            void slam_pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr data);

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ndt_odom_;
            rclcpp::TimerBase::SharedPtr timer_;

            sensor_msgs::msg::PointCloud2 velodyne_;
            sensor_msgs::msg::PointCloud2 map_;

    };
}


#endif