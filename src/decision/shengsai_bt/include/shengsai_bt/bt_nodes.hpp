#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "shengsai_bt/shengsai_bt_node.hpp"

namespace shengsai_bt {

class SetSafeMode : public BT::SyncActionNode
{
public:
  SetSafeMode(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  ShengsaiBtNode & node_;
};

class WaitForSystemReady : public BT::StatefulActionNode
{
public:
  WaitForSystemReady(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  ShengsaiBtNode & node_;
  double timeout_s_{60.0};
  double offline_stale_s_{2.0};
  std::chrono::steady_clock::time_point start_time_{};
};

class NavigateToGoal : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::Client<ShengsaiBtNode::NavigateToPose>::GoalHandle;
  using WrappedResult = rclcpp_action::Client<ShengsaiBtNode::NavigateToPose>::WrappedResult;

  NavigateToGoal(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class AttemptState
  {
    WAIT_SYSTEM_READY,
    SEND_GOAL,
    WAIT_GOAL_HANDLE,
    WAIT_RESULT,
    WAIT_RETRY,
  };

  void startAttempt();
  BT::NodeStatus handleAttemptFailure(const std::string & reason);
  void cancelCurrentGoal();
  bool attemptTimedOut() const;
  void feedbackCallback(const std::shared_ptr<const ShengsaiBtNode::NavigateToPose::Feedback> & feedback);
  bool feedbackNearGoal(double & distance, double & yaw_error) const;
  void resetFeedback();
  void resetNearGoalDwell();

  ShengsaiBtNode & node_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double timeout_s_{120.0};
  int max_retries_{2};
  int attempt_count_{0};
  bool cancel_sent_{false};
  bool require_system_ready_{false};
  double system_ready_timeout_s_{120.0};
  double offline_stale_s_{2.0};
  double arrival_xy_tolerance_{0.20};
  double arrival_yaw_tolerance_{0.15};
  double feedback_stale_s_{2.0};
  double arrival_dwell_s_{0.5};
  bool near_goal_dwell_active_{false};
  std::chrono::steady_clock::time_point near_goal_dwell_start_time_{};
  std::string phase_;
  AttemptState attempt_state_{AttemptState::SEND_GOAL};
  std::chrono::steady_clock::time_point attempt_start_time_{};
  std::chrono::steady_clock::time_point system_ready_start_time_{};
  std::chrono::steady_clock::time_point next_retry_time_{};

  mutable std::mutex feedback_mutex_;
  geometry_msgs::msg::PoseStamped last_feedback_pose_;
  std::chrono::steady_clock::time_point last_feedback_time_{};
  bool has_feedback_{false};

  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  std::shared_future<WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;
};

class EngageAtTarget : public BT::StatefulActionNode
{
public:
  EngageAtTarget(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class State
  {
    WAIT_ATTACK_START,
    ENGAGE,
  };

  ShengsaiBtNode & node_;
  double target_wait_timeout_s_{20.0};
  double engage_timeout_s_{20.0};
  double offline_stale_s_{2.0};
  State state_{State::WAIT_ATTACK_START};
  std::chrono::steady_clock::time_point state_start_time_{};
};

class WaitForTarget : public BT::StatefulActionNode
{
public:
  WaitForTarget(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  ShengsaiBtNode & node_;
  double timeout_s_{60.0};
  double offline_stale_s_{2.0};
  std::chrono::steady_clock::time_point start_time_{};
};

class EngageUntilTargetLost : public BT::StatefulActionNode
{
public:
  EngageUntilTargetLost(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  ShengsaiBtNode & node_;
  double timeout_s_{180.0};
  double offline_stale_s_{2.0};
  std::chrono::steady_clock::time_point start_time_{};
};

class CompleteMission : public BT::StatefulActionNode
{
public:
  CompleteMission(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  ShengsaiBtNode & node_;
};

class FailAndHold : public BT::StatefulActionNode
{
public:
  FailAndHold(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  ShengsaiBtNode & node_;
};

}  // namespace shengsai_bt
