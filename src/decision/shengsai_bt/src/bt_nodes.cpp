#include "shengsai_bt/bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

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
  attempt_count_ = 0;
  node_.setSafeMode(phase_);
  startAttempt();
  return onRunning();
}

void NavigateToGoal::startAttempt()
{
  ++attempt_count_;
  cancel_sent_ = false;
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

    goal_handle_future_ = client->async_send_goal(goal);
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

  goal_handle_.reset();
  goal_handle_future_ = {};
  result_future_ = {};
  cancel_sent_ = false;

  if (attempt_count_ <= max_retries_) {
    attempt_state_ = AttemptState::WAIT_RETRY;
    next_retry_time_ = std::chrono::steady_clock::now() + 1s;
    return BT::NodeStatus::RUNNING;
  }

  node_.reportFailure(phase_ + " navigation failed after all attempts");
  return BT::NodeStatus::FAILURE;
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
  node_.setSafeMode("WAIT_TARGET");
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
  node_.setCombatMode();
  if (!node_.hasTargetFinished()) {
    RCLCPP_INFO(node_.get_logger(), "Auto aim enabled: not_aim_enemy=false.");
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
