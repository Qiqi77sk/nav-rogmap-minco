#include "shengsai_bt/bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace shengsai_bt {
namespace {
double elapsedSeconds(const std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

ShengsaiBtNode & nodeFromConfig(const BT::NodeConfiguration & config)
{
  auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto * typed_node = dynamic_cast<ShengsaiBtNode *>(node.get());
  if (typed_node == nullptr) {
    throw std::runtime_error("Behavior-tree blackboard does not contain ShengsaiBtNode");
  }
  return *typed_node;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

SetSafeMode::SetSafeMode(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList SetSafeMode::providedPorts()
{
  return {};
}

BT::NodeStatus SetSafeMode::tick()
{
  node_.setSafeMode("SAFE_MODE");
  return BT::NodeStatus::SUCCESS;
}

WaitForSystemReady::WaitForSystemReady(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList WaitForSystemReady::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_s", 60.0, "Maximum time to wait for navigation and offline info"),
    BT::InputPort<double>("offline_info_stale_s", 2.0, "Maximum accepted offline-info age"),
  };
}

BT::NodeStatus WaitForSystemReady::onStart()
{
  timeout_s_ = getInput<double>("timeout_s").value_or(60.0);
  offline_stale_s_ = getInput<double>("offline_info_stale_s").value_or(2.0);
  start_time_ = std::chrono::steady_clock::now();
  node_.setSafeMode("WAIT_SYSTEM_READY");
  return onRunning();
}

BT::NodeStatus WaitForSystemReady::onRunning()
{
  if (node_.isNavigationServerReady() && node_.hasFreshOfflineInfo(offline_stale_s_)) {
    RCLCPP_INFO(node_.get_logger(), "Navigation action server and offline info are ready.");
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_s_ > 0.0 && elapsedSeconds(start_time_) >= timeout_s_) {
    node_.reportFailure("system ready timeout");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForSystemReady::onHalted()
{
}

NavigateToGoal::NavigateToGoal(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList NavigateToGoal::providedPorts()
{
  return {
    BT::InputPort<double>("goal_x", "Goal x in the map frame"),
    BT::InputPort<double>("goal_y", "Goal y in the map frame"),
    BT::InputPort<double>("goal_yaw", "Goal yaw in radians"),
    BT::InputPort<double>("timeout_s", 120.0, "Timeout for one navigation attempt"),
    BT::InputPort<int>("max_retries", 2, "Retries after the initial navigation attempt"),
    BT::InputPort<std::string>("phase", "GO_TO_TARGET", "Mission phase name for logging"),
    BT::InputPort<bool>("require_system_ready",
      false,
      "Wait for Nav2 and fresh offline info before the first navigation attempt"),
    BT::InputPort<double>(
      "system_ready_timeout_s", 120.0, "Maximum time to wait for Nav2 and offline info"),
    BT::InputPort<double>(
      "offline_info_stale_s", 2.0, "Maximum accepted offline-info age for the readiness check"),
    BT::InputPort<double>("arrival_xy_tolerance",
      0.20,
      "Fallback arrival xy tolerance when the navigation action ends near the goal"),
    BT::InputPort<double>("arrival_yaw_tolerance",
      0.15,
      "Fallback arrival yaw tolerance when the navigation action ends near the goal"),
    BT::InputPort<double>("feedback_stale_s", 2.0, "Maximum feedback age for the fallback arrival check"),
    BT::InputPort<double>("arrival_dwell_s",
      0.5,
      "Feedback must stay within the arrival tolerance this long before canceling an active goal"),
  };
}

BT::NodeStatus NavigateToGoal::onStart()
{
  const auto goal_x = getInput<double>("goal_x");
  const auto goal_y = getInput<double>("goal_y");
  const auto goal_yaw = getInput<double>("goal_yaw");
  if (!goal_x || !goal_y || !goal_yaw) {
    node_.reportFailure("navigation goal input is missing");
    return BT::NodeStatus::FAILURE;
  }

  goal_x_ = goal_x.value();
  goal_y_ = goal_y.value();
  goal_yaw_ = goal_yaw.value();
  timeout_s_ = getInput<double>("timeout_s").value_or(120.0);
  max_retries_ = std::max(0, getInput<int>("max_retries").value_or(2));
  phase_ = getInput<std::string>("phase").value_or("NAVIGATING");
  require_system_ready_ = getInput<bool>("require_system_ready").value_or(false);
  system_ready_timeout_s_ = getInput<double>("system_ready_timeout_s").value_or(120.0);
  offline_stale_s_ = getInput<double>("offline_info_stale_s").value_or(2.0);
  arrival_xy_tolerance_ = getInput<double>("arrival_xy_tolerance").value_or(0.20);
  arrival_yaw_tolerance_ = getInput<double>("arrival_yaw_tolerance").value_or(0.15);
  feedback_stale_s_ = getInput<double>("feedback_stale_s").value_or(2.0);
  arrival_dwell_s_ = getInput<double>("arrival_dwell_s").value_or(0.5);
  attempt_count_ = 0;
  node_.setSafeMode(require_system_ready_ ? "WAIT_SYSTEM_READY" : phase_);
  node_.setZeroHold(false);

  if (require_system_ready_) {
    attempt_state_ = AttemptState::WAIT_SYSTEM_READY;
    system_ready_start_time_ = std::chrono::steady_clock::now();
    return onRunning();
  }

  startAttempt();
  return onRunning();
}

void NavigateToGoal::startAttempt()
{
  ++attempt_count_;
  cancel_sent_ = false;
  resetFeedback();
  resetNearGoalDwell();
  node_.setSafeMode(phase_);
  node_.setZeroHold(false);
  goal_handle_.reset();
  goal_handle_future_ = {};
  result_future_ = {};
  attempt_start_time_ = std::chrono::steady_clock::now();
  attempt_state_ = AttemptState::SEND_GOAL;

  RCLCPP_INFO(node_.get_logger(),
    "%s navigation attempt %d/%d to map (%.2f, %.2f, yaw=%.3f)",
    phase_.c_str(),
    attempt_count_,
    max_retries_ + 1,
    goal_x_,
    goal_y_,
    goal_yaw_);
}

BT::NodeStatus NavigateToGoal::onRunning()
{
  auto client = node_.navigationClient();
  if (!client) {
    node_.reportFailure("navigate_to_pose client is unavailable");
    return BT::NodeStatus::FAILURE;
  }

  switch (attempt_state_) {
  case AttemptState::WAIT_SYSTEM_READY: {
    if (node_.isNavigationServerReady() && node_.hasFreshOfflineInfo(offline_stale_s_)) {
      RCLCPP_INFO(node_.get_logger(), "Navigation action server and offline info are ready.");
      startAttempt();
      return onRunning();
    }
    if (system_ready_timeout_s_ > 0.0 &&
        elapsedSeconds(system_ready_start_time_) >= system_ready_timeout_s_) {
      node_.reportFailure("system ready timeout");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  case AttemptState::SEND_GOAL: {
    if (attemptTimedOut()) {
      return handleAttemptFailure("navigation action server unavailable");
    }
    if (!node_.isNavigationServerReady()) {
      return BT::NodeStatus::RUNNING;
    }

    ShengsaiBtNode::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_.now();
    goal.pose.pose.position.x = goal_x_;
    goal.pose.pose.position.y = goal_y_;
    const double half_yaw = goal_yaw_ * 0.5;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(half_yaw);
    goal.pose.pose.orientation.w = std::cos(half_yaw);

    rclcpp_action::Client<ShengsaiBtNode::NavigateToPose>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandle::SharedPtr,
        const std::shared_ptr<const ShengsaiBtNode::NavigateToPose::Feedback> feedback) {
        feedbackCallback(feedback);
      };
    goal_handle_future_ = client->async_send_goal(goal, options);
    attempt_state_ = AttemptState::WAIT_GOAL_HANDLE;
    return BT::NodeStatus::RUNNING;
  }

  case AttemptState::WAIT_GOAL_HANDLE: {
    if (goal_handle_future_.valid() && goal_handle_future_.wait_for(0s) == std::future_status::ready) {
      try {
        goal_handle_ = goal_handle_future_.get();
      } catch (const std::exception & error) {
        return handleAttemptFailure(std::string("goal request exception: ") + error.what());
      }
      if (!goal_handle_) {
        return handleAttemptFailure("navigate_to_pose rejected the goal");
      }
      try {
        result_future_ = client->async_get_result(goal_handle_);
      } catch (const std::exception & error) {
        return handleAttemptFailure(std::string("result request exception: ") + error.what());
      }
      attempt_state_ = AttemptState::WAIT_RESULT;
      return BT::NodeStatus::RUNNING;
    }
    if (attemptTimedOut()) {
      return handleAttemptFailure("goal response timeout");
    }
    return BT::NodeStatus::RUNNING;
  }

  case AttemptState::WAIT_RESULT: {
    if (result_future_.valid() && result_future_.wait_for(0s) == std::future_status::ready) {
      try {
        const auto & result = result_future_.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          // Stop the chassis immediately on arrival and keep /cmd_vel_mpc at zero
          // until the next navigation attempt explicitly releases the hold.
          node_.setZeroHold(true);
          RCLCPP_INFO(node_.get_logger(), "%s navigation reached the goal.", phase_.c_str());
          return BT::NodeStatus::SUCCESS;
        }
        std::ostringstream reason;
        reason << "navigation ended with result code " << static_cast<int>(result.code);
        return handleAttemptFailure(reason.str());
      } catch (const std::exception & error) {
        return handleAttemptFailure(std::string("result exception: ") + error.what());
      }
    }

    double feedback_distance = 0.0;
    double feedback_yaw_error = 0.0;
    if (feedbackNearGoal(feedback_distance, feedback_yaw_error)) {
      if (!near_goal_dwell_active_) {
        near_goal_dwell_active_ = true;
        near_goal_dwell_start_time_ = std::chrono::steady_clock::now();
      } else if (arrival_dwell_s_ <= 0.0 ||
                 elapsedSeconds(near_goal_dwell_start_time_) >= arrival_dwell_s_) {
        RCLCPP_WARN(node_.get_logger(),
          "%s feedback pose is within arrival tolerance while navigation is still running "
          "(dist=%.3f m, yaw_err=%.3f rad); canceling goal and treating as arrival.",
          phase_.c_str(),
          feedback_distance,
          feedback_yaw_error);
        cancelCurrentGoal();
        node_.setZeroHold(true);
        goal_handle_.reset();
        goal_handle_future_ = {};
        result_future_ = {};
        cancel_sent_ = false;
        resetNearGoalDwell();
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      resetNearGoalDwell();
    }

    if (attemptTimedOut()) {
      cancelCurrentGoal();
      if (elapsedSeconds(attempt_start_time_) >= timeout_s_ + 2.0) {
        return handleAttemptFailure("navigation timeout");
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  case AttemptState::WAIT_RETRY:
    if (std::chrono::steady_clock::now() >= next_retry_time_) {
      startAttempt();
      return onRunning();
    }
    return BT::NodeStatus::RUNNING;
  }

  node_.reportFailure("navigation state machine reached an invalid state");
  return BT::NodeStatus::FAILURE;
}

bool NavigateToGoal::attemptTimedOut() const
{
  return timeout_s_ > 0.0 && elapsedSeconds(attempt_start_time_) >= timeout_s_;
}

BT::NodeStatus NavigateToGoal::handleAttemptFailure(const std::string & reason)
{
  RCLCPP_WARN(node_.get_logger(),
    "%s navigation attempt %d failed: %s",
    phase_.c_str(),
    attempt_count_,
    reason.c_str());

  node_.setZeroHold(true);
  goal_handle_.reset();
  goal_handle_future_ = {};
  result_future_ = {};
  cancel_sent_ = false;

  double feedback_distance = 0.0;
  double feedback_yaw_error = 0.0;
  if (feedbackNearGoal(feedback_distance, feedback_yaw_error)) {
    RCLCPP_WARN(node_.get_logger(),
      "%s navigation failed (%s), but feedback pose is within arrival tolerance "
      "(dist=%.3f m, yaw_err=%.3f rad); treating as arrival.",
      phase_.c_str(),
      reason.c_str(),
      feedback_distance,
      feedback_yaw_error);
    return BT::NodeStatus::SUCCESS;
  }

  if (attempt_count_ <= max_retries_) {
    attempt_state_ = AttemptState::WAIT_RETRY;
    next_retry_time_ = std::chrono::steady_clock::now() + 1s;
    return BT::NodeStatus::RUNNING;
  }

  node_.reportFailure(phase_ + " navigation failed after all attempts");
  return BT::NodeStatus::FAILURE;
}

void NavigateToGoal::feedbackCallback(
  const std::shared_ptr<const ShengsaiBtNode::NavigateToPose::Feedback> & feedback)
{
  if (!feedback) {
    return;
  }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  last_feedback_pose_ = feedback->current_pose;
  last_feedback_time_ = std::chrono::steady_clock::now();
  has_feedback_ = true;
}

bool NavigateToGoal::feedbackNearGoal(double & distance, double & yaw_error) const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!has_feedback_) {
    return false;
  }
  if (feedback_stale_s_ > 0.0 && elapsedSeconds(last_feedback_time_) > feedback_stale_s_) {
    return false;
  }

  const auto & pose = last_feedback_pose_;
  // Nav2 NavigateToPose feedback is in the global frame. Only accept the
  // fallback when the frame is map or unspecified.
  if (!pose.header.frame_id.empty() && pose.header.frame_id != "map") {
    return false;
  }

  distance = std::hypot(pose.pose.position.x - goal_x_, pose.pose.position.y - goal_y_);

  // Nav2 feedback 的 current_pose 来自 map->base_link；Point-LIO 发布的
  // camera_init->base_link 旋转恒为单位四元数，因此 feedback yaw 不代表底盘朝向。
  // 到点 yaw 改用 /sentry/offline_info.chassis_yaw_camerainit，并把 map 下的 goal yaw
  // 转到 camera_init 后比较。
  double chassis_yaw_camera_init = 0.0;
  if (!node_.getFreshChassisYawCameraInit(chassis_yaw_camera_init, 0.25)) {
    return false;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = rclcpp::Time(0);
  goal_pose.pose.position.x = goal_x_;
  goal_pose.pose.position.y = goal_y_;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.x = 0.0;
  goal_pose.pose.orientation.y = 0.0;
  goal_pose.pose.orientation.z = std::sin(goal_yaw_ * 0.5);
  goal_pose.pose.orientation.w = std::cos(goal_yaw_ * 0.5);

  geometry_msgs::msg::PoseStamped goal_in_camera_init;
  try {
    goal_in_camera_init = node_.tfBuffer()->transform(goal_pose, "camera_init");
  } catch (const tf2::TransformException &) {
    return false;
  }

  const double goal_yaw_camera_init = yawFromQuaternion(goal_in_camera_init.pose.orientation);
  yaw_error = std::atan2(
    std::sin(goal_yaw_camera_init - chassis_yaw_camera_init),
    std::cos(goal_yaw_camera_init - chassis_yaw_camera_init));
  return distance <= arrival_xy_tolerance_ && std::abs(yaw_error) <= arrival_yaw_tolerance_;
}

void NavigateToGoal::resetFeedback()
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  has_feedback_ = false;
  last_feedback_pose_ = geometry_msgs::msg::PoseStamped();
  last_feedback_time_ = std::chrono::steady_clock::time_point{};
}

void NavigateToGoal::resetNearGoalDwell()
{
  near_goal_dwell_active_ = false;
  near_goal_dwell_start_time_ = std::chrono::steady_clock::time_point{};
}

void NavigateToGoal::cancelCurrentGoal()
{
  if (cancel_sent_ || !goal_handle_) {
    return;
  }
  cancel_sent_ = true;
  try {
    node_.navigationClient()->async_cancel_goal(goal_handle_);
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_.get_logger(), "Failed to cancel navigation goal: %s", error.what());
  }
}

void NavigateToGoal::onHalted()
{
  cancelCurrentGoal();
  node_.requestStop();
}

EngageAtTarget::EngageAtTarget(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList EngageAtTarget::providedPorts()
{
  return {
    BT::InputPort<double>("target_wait_timeout_s", 20.0, "Maximum time to wait for is_get=true"),
    BT::InputPort<double>("engage_timeout_s", 20.0, "Maximum engagement duration"),
    BT::InputPort<double>("offline_info_stale_s", 2.0, "Maximum accepted offline-info age"),
  };
}

BT::NodeStatus EngageAtTarget::onStart()
{
  target_wait_timeout_s_ = getInput<double>("target_wait_timeout_s").value_or(20.0);
  engage_timeout_s_ = getInput<double>("engage_timeout_s").value_or(20.0);
  offline_stale_s_ = getInput<double>("offline_info_stale_s").value_or(2.0);

  if (node_.hasTargetFinished()) {
    RCLCPP_WARN(node_.get_logger(),
      "EngageAtTarget entered after the target was already finished; skipping engagement.");
    node_.setZeroHold(true);
    return BT::NodeStatus::SUCCESS;
  }

  node_.setCombatMode("WAIT_TARGET");
  node_.beginAttackTracking();
  node_.setZeroHold(true);

  state_ = State::WAIT_ATTACK_START;
  state_start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_.get_logger(),
    "Reached target: not_aim_enemy=false, stance=ATTACK, chassis zero hold active. "
    "Waiting for is_get=true.");
  return onRunning();
}

BT::NodeStatus EngageAtTarget::onRunning()
{
  if (offline_stale_s_ > 0.0 && node_.offlineInfoAgeSeconds() > offline_stale_s_) {
    node_.setSafeMode("ATTACK_STALE");
    RCLCPP_WARN(
      node_.get_logger(), "Offline target info became stale during attack; ending attack and returning.");
    return BT::NodeStatus::SUCCESS;
  }

  switch (state_) {
  case State::WAIT_ATTACK_START:
    if (node_.attackStarted()) {
      state_ = State::ENGAGE;
      state_start_time_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(node_.get_logger(), "Attack started: is_get=true. Entering engagement.");
      return onRunning();
    }
    if (target_wait_timeout_s_ > 0.0 && elapsedSeconds(state_start_time_) >= target_wait_timeout_s_) {
      node_.setSafeMode("ATTACK_START_TIMEOUT");
      RCLCPP_WARN(node_.get_logger(), "Attack start timeout; ending attack and returning.");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;

  case State::ENGAGE:
    if (node_.attackFinished()) {
      node_.finishCombatAndLatch();
      RCLCPP_INFO(node_.get_logger(), "Attack finished: is_get=false. Auto aim disabled; returning.");
      return BT::NodeStatus::SUCCESS;
    }
    if (engage_timeout_s_ > 0.0 && elapsedSeconds(state_start_time_) >= engage_timeout_s_) {
      node_.setSafeMode("ATTACK_TIMEOUT");
      RCLCPP_WARN(node_.get_logger(), "Attack execution timeout; ending attack and returning.");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  node_.reportFailure("engagement state machine reached an invalid state");
  return BT::NodeStatus::FAILURE;
}

void EngageAtTarget::onHalted()
{
  node_.setSafeMode("ENGAGEMENT_HALTED");
  node_.setZeroHold(true);
}

WaitForTarget::WaitForTarget(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList WaitForTarget::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_s", 60.0, "Maximum time to wait for is_get=true"),
    BT::InputPort<double>("offline_info_stale_s", 2.0, "Maximum accepted offline-info age"),
  };
}

BT::NodeStatus WaitForTarget::onStart()
{
  timeout_s_ = getInput<double>("timeout_s").value_or(60.0);
  offline_stale_s_ = getInput<double>("offline_info_stale_s").value_or(2.0);
  start_time_ = std::chrono::steady_clock::now();
  // 到达终点后立即允许下位机自瞄，再等待视觉确认 is_get=true。
  node_.setCombatMode("WAIT_TARGET");
  RCLCPP_INFO(node_.get_logger(), "Reached target: not_aim_enemy=false. Waiting for is_get=true.");
  return onRunning();
}

BT::NodeStatus WaitForTarget::onRunning()
{
  if (node_.armTargetTrackingIfAcquired(offline_stale_s_)) {
    RCLCPP_INFO(node_.get_logger(), "Target acquired: is_get=true. Entering engagement.");
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_s_ > 0.0 && elapsedSeconds(start_time_) >= timeout_s_) {
    node_.reportFailure("target wait timeout");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForTarget::onHalted()
{
}

EngageUntilTargetLost::EngageUntilTargetLost(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList EngageUntilTargetLost::providedPorts()
{
  return {
    BT::InputPort<double>("timeout_s", 180.0, "Maximum engagement duration"),
    BT::InputPort<double>("offline_info_stale_s", 2.0, "Maximum accepted offline-info age"),
  };
}

BT::NodeStatus EngageUntilTargetLost::onStart()
{
  timeout_s_ = getInput<double>("timeout_s").value_or(180.0);
  offline_stale_s_ = getInput<double>("offline_info_stale_s").value_or(2.0);
  start_time_ = std::chrono::steady_clock::now();
  node_.setCombatMode("ENGAGE");
  if (!node_.hasTargetFinished()) {
    RCLCPP_INFO(node_.get_logger(), "Engagement active: waiting for is_get=false.");
  }
  return onRunning();
}

BT::NodeStatus EngageUntilTargetLost::onRunning()
{
  const double info_age_s = node_.offlineInfoAgeSeconds();
  if (offline_stale_s_ > 0.0 && info_age_s > offline_stale_s_) {
    node_.reportFailure("offline target info became stale during engagement");
    return BT::NodeStatus::FAILURE;
  }

  if (node_.hasTargetFinished() || !node_.isTargetAcquired()) {
    node_.finishCombatAndLatch();
    RCLCPP_INFO(node_.get_logger(), "Target finished: is_get=false. Auto aim disabled and latched.");
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_s_ > 0.0 && elapsedSeconds(start_time_) >= timeout_s_) {
    node_.reportFailure("engagement timeout");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void EngageUntilTargetLost::onHalted()
{
  node_.setSafeMode("ENGAGEMENT_HALTED");
}

CompleteMission::CompleteMission(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList CompleteMission::providedPorts()
{
  return {};
}

BT::NodeStatus CompleteMission::onStart()
{
  node_.markMissionComplete();
  RCLCPP_INFO(node_.get_logger(), "Shengsai mission complete. Holding safe state.");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CompleteMission::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void CompleteMission::onHalted()
{
}

FailAndHold::FailAndHold(const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config), node_(nodeFromConfig(config))
{
}

BT::PortsList FailAndHold::providedPorts()
{
  return {};
}

BT::NodeStatus FailAndHold::onStart()
{
  node_.reportFailure("mission sequence failed");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FailAndHold::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void FailAndHold::onHalted()
{
}

}  // namespace shengsai_bt
