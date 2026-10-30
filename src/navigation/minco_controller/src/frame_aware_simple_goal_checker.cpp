#include "minco_controller/frame_aware_simple_goal_checker.hpp"

#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_controller
{

void FrameAwareSimpleGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_controller::SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);

  node_ = parent.lock();
  if (!node_) {
    return;
  }

  // The robot/query pose comes from the local costmap (camera_init).
  query_frame_ = costmap_ros ? costmap_ros->getGlobalFrameID() : std::string("camera_init");

  // The goal pose comes from the planner output (map).
  if (!node_->has_parameter(plugin_name + ".goal_frame")) {
    node_->declare_parameter(plugin_name + ".goal_frame", std::string("map"));
  }
  node_->get_parameter(plugin_name + ".goal_frame", goal_frame_);
  if (goal_frame_.empty()) {
    goal_frame_ = "map";
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // base_link 的 yaw 来自 Point-LIO，雷达位于不随底盘自旋的云台上，
  // 因此 goal checker 必须使用底盘 IMU yaw，而不是 query_pose 的 yaw。
  chassis_info_sub_ = node_->create_subscription<ros_interfaces::msg::SentryInfoOffline>(
    "/sentry/offline_info", 1,
    [this](const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg) {
      if (!msg || !std::isfinite(msg->chassis_yaw_camerainit)) {
        return;
      }
      std::lock_guard<std::mutex> lock(chassis_yaw_mutex_);
      chassis_yaw_camerainit_ = msg->chassis_yaw_camerainit;
      chassis_yaw_rx_ = std::chrono::steady_clock::now();
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "[FrameAwareGoalChecker] goal_frame=%s query_frame=%s",
    goal_frame_.c_str(), query_frame_.c_str());
}

bool FrameAwareSimpleGoalChecker::getFreshChassisYaw(double & yaw) const
{
  std::lock_guard<std::mutex> lock(chassis_yaw_mutex_);
  if (!std::isfinite(chassis_yaw_camerainit_) ||
      chassis_yaw_rx_ == std::chrono::steady_clock::time_point{}) {
    return false;
  }
  const double age_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - chassis_yaw_rx_).count();
  if (age_s > 0.25) {
    return false;
  }
  yaw = chassis_yaw_camerainit_;
  return true;
}

bool FrameAwareSimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!tf_buffer_) {
    return nav2_controller::SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  }

  geometry_msgs::msg::Pose goal_in_query_frame = goal_pose;
  if (goal_frame_ != query_frame_) {
    geometry_msgs::msg::PoseStamped goal_in;
    goal_in.header.frame_id = goal_frame_;
    // 0 表示取最新 TF；带 now() 时间戳的查询会因动态 TF 最新时间早于 now 而外插失败。
    goal_in.header.stamp = rclcpp::Time(0);
    goal_in.pose = goal_pose;

    geometry_msgs::msg::PoseStamped goal_out;
    try {
      goal_out = tf_buffer_->transform(goal_in, query_frame_);
    } catch (const tf2::TransformException &) {
      // A failed frame conversion must never be treated as "goal reached".
      return false;
    }
    goal_in_query_frame = goal_out.pose;
  }

  double chassis_yaw = 0.0;
  if (!getFreshChassisYaw(chassis_yaw)) {
    return false;
  }
  geometry_msgs::msg::Pose query_with_chassis_yaw = query_pose;
  tf2::Quaternion heading_q;
  heading_q.setRPY(0.0, 0.0, chassis_yaw);
  query_with_chassis_yaw.orientation = tf2::toMsg(heading_q);

  return nav2_controller::SimpleGoalChecker::isGoalReached(
    query_with_chassis_yaw, goal_in_query_frame, velocity);
}

}  // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::FrameAwareSimpleGoalChecker, nav2_core::GoalChecker)
