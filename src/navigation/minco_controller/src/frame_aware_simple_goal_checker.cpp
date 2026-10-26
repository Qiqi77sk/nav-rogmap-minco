#include "minco_controller/frame_aware_simple_goal_checker.hpp"

#include <string>

#include "pluginlib/class_list_macros.hpp"
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

  RCLCPP_INFO(
    node_->get_logger(),
    "[FrameAwareGoalChecker] goal_frame=%s query_frame=%s",
    goal_frame_.c_str(), query_frame_.c_str());
}

bool FrameAwareSimpleGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!tf_buffer_ || goal_frame_ == query_frame_) {
    return nav2_controller::SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  }

  geometry_msgs::msg::PoseStamped goal_in;
  goal_in.header.frame_id = goal_frame_;
  goal_in.header.stamp = node_ ? node_->now() : rclcpp::Time(0);
  goal_in.pose = goal_pose;

  geometry_msgs::msg::PoseStamped goal_out;
  try {
    goal_out = tf_buffer_->transform(goal_in, query_frame_);
  } catch (const tf2::TransformException &) {
    // A failed frame conversion must never be treated as "goal reached".
    return false;
  }

  return nav2_controller::SimpleGoalChecker::isGoalReached(
    query_pose, goal_out.pose, velocity);
}

}  // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::FrameAwareSimpleGoalChecker, nav2_core::GoalChecker)
