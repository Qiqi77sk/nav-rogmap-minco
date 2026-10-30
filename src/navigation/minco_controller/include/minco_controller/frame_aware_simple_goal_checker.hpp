#pragma once

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace minco_controller
{

/**
 * @brief SimpleGoalChecker with frame conversion before the XY/yaw comparison.
 *
 * The Nav2 controller_server passes the robot pose in the local costmap frame
 * (camera_init) and the path goal in the planner output frame (map). This
 * checker transforms the goal into the robot/query frame first, so both poses
 * are compared in the same coordinate system.
 */
class FrameAwareSimpleGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  bool getFreshChassisYaw(double & yaw) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<ros_interfaces::msg::SentryInfoOffline>::SharedPtr chassis_info_sub_;
  mutable std::mutex chassis_yaw_mutex_;
  double chassis_yaw_camerainit_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point chassis_yaw_rx_{};
  std::string goal_frame_{"map"};
  std::string query_frame_{"camera_init"};
};

}  // namespace minco_controller
