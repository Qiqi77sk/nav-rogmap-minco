#pragma once

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace shengsai_bt {

class ShengsaiBtNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  ShengsaiBtNode();

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigationClient() const;
  bool isNavigationServerReady() const;
  bool isTargetAcquired() const;
  bool hasTargetFinished() const;
  bool armTargetTrackingIfAcquired(double max_age_s);
  bool hasFreshOfflineInfo(double max_age_s) const;
  double offlineInfoAgeSeconds() const;
  // 底盘 IMU yaw（camera_init 系）；NaN 或过期时返回 false。
  bool getFreshChassisYawCameraInit(double & yaw, double max_age_s = 0.25) const;
  const std::shared_ptr<tf2_ros::Buffer> & tfBuffer() const;

  void populateBlackboard(BT::Blackboard & blackboard) const;
  void setSafeMode(const std::string & phase);
  void setCombatMode(const std::string & phase);
  void finishCombatAndLatch();
  void markMissionComplete();
  void reportFailure(const std::string & reason);
  void requestStop();

  void publishChassisZero();
  void setZeroHold(bool active);
  void beginAttackTracking();
  void endAttackTracking();
  bool attackStarted() const;
  bool attackFinished() const;

private:
  void offlineInfoCallback(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg);
  void publishBehavior();
  void publishStopCommand();
  void setPhase(const std::string & phase);

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Subscription<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_info_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ros_interfaces::msg::Behavior>::SharedPtr behavior_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_mpc_pub_;
  rclcpp::TimerBase::SharedPtr behavior_timer_;

  std::atomic<bool> aim_allowed_{false};
  std::atomic<bool> combat_active_{false};
  std::atomic<bool> target_finished_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> failure_reported_{false};
  std::atomic<bool> zero_hold_active_{false};

  mutable std::mutex offline_info_mutex_;
  bool has_offline_info_{false};
  bool is_get_{false};
  bool target_tracking_armed_{false};
  bool attack_phase_active_{false};
  bool attack_started_{false};
  bool attack_finished_{false};
  std::chrono::steady_clock::time_point last_offline_info_time_{};
  double chassis_yaw_camerainit_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point last_chassis_yaw_time_{};

  mutable std::mutex phase_mutex_;
  std::string phase_{"WAIT_SYSTEM_READY"};
};

}  // namespace shengsai_bt
