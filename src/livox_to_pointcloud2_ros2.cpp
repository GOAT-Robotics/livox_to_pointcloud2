#include <livox_to_pointcloud2/livox_to_pointcloud2_ros2.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#ifdef LIVOX_ROS2_DRIVER
#include <livox_interfaces/msg/custom_msg.hpp>
#endif

#ifdef LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace livox_to_pointcloud2 {

LivoxToPointCloud2::LivoxToPointCloud2(const rclcpp::NodeOptions& options)
: rclcpp::Node("livox_to_pointcloud2", options)
{
  // BEST_EFFORT publisher for sensor streams
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

  points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar/pcl2",
    qos
  );

auto in_qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable().durability_volatile();

#ifdef LIVOX_ROS2_DRIVER
  livox_sub = this->create_subscription<livox_interfaces::msg::CustomMsg>(
    "/livox/lidar",
    in_qos,
    [this](const livox_interfaces::msg::CustomMsg::ConstSharedPtr livox_msg) {
      const auto points_msg = converter.convert(*livox_msg);
      points_pub->publish(*points_msg);
    }
  );
#endif

#ifdef LIVOX_ROS_DRIVER2
  livox2_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    "/livox/lidar",
    in_qos,
    [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_msg) {
      const auto points_msg = converter.convert(*livox_msg);
      points_pub->publish(*points_msg);
    }
  );
#endif
}


LivoxToPointCloud2::~LivoxToPointCloud2() {}

}  // namespace livox_to_pointcloud2

RCLCPP_COMPONENTS_REGISTER_NODE(livox_to_pointcloud2::LivoxToPointCloud2);