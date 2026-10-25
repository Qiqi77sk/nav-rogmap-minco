#include "shengsai_bt/shengsai_bt_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

using namespace std::chrono_literals;

namespace shengsai_bt {
namespace {
constexpr uint8_t kStanceAttack = 1;
constexpr uint8_t kStanceMove = 3;
constexpr uint8_t kControlAuto = 0;
constexpr uint8_t kLifterTop = 0;
constexpr uint8_t kPitchBottom = 1;
}  // namespace

ShengsaiBtNode::ShengsaiBtNode() : Node("shengsai_bt")
{
  declare_parameter<double>("bt_tick_hz", 20.0);
  declare_parameter<double>("behavior_publish_hz", 10.0);
  declare_parameter<double>("start_x", 11.56);
  declare_parameter<double>("start_y", 5.26);
  declare_parameter<double>("start_yaw_rad", 3.141592653589793);
  declare_parameter<double>("target_x", 3.83);
  declare_parameter<double>("target_y", 3.44);
  declare_parameter<double>("target_yaw_rad", 3.141592653589793);
  declare_parameter<double>("system_ready_timeout_s", 60.0);
  declare_parameter<double>("offline_info_stale_s", 2.0);
  declare_parameter<double>("navigation_timeout_s", 120.0);
  declare_parameter<int>("navigation_retries", 2);
  declare_parameter<double>("target_wait_timeout_s", 60.0);
  declare_parameter<double>("engage_timeout_s", 180.0);

  navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  offline_info_sub_ = create_subscription<ros_interfaces::msg::SentryInfoOffline>(
    "/sentry/offline_info", 1, [this](const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg) {
      offlineInfoCallback(msg);
    });

  behavior_pub_ = create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  const double configured_hz = get_parameter("behavior_publish_hz").as_double();
  const double publish_hz = std::clamp(configured_hz, 1.0, 100.0);
  const auto period =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / publish_hz));
  behavior_timer_ = create_wall_timer(period, [this]() {
    publishBehavior();
  });

  publishBehavior();
}

rclcpp_action::Client<ShengsaiBtNode::NavigateToPose>::SharedPtr ShengsaiBtNode::navigationClient() const
{
  return navigation_client_;
}

bool ShengsaiBtNode::isNavigationServerReady() const
{
  return navigation_client_ && navigation_client_->action_server_is_ready();
}

bool ShengsaiBtNode::isTargetAcquired() const
{
  std::lock_guard<std::mutex> lock(offline_info_mutex_);
  return is_get_;
}

bool ShengsaiBtNode::hasTargetFinished() const
{
  return target_finished_.load();
}

bool ShengsaiBtNode::armTargetTrackingIfAcquired(const double max_age_s)
{
  std::lock_guard<std::mutex> lock(offline_info_mutex_);
  if (!has_offline_info_ || !is_get_) {
    return false;
  }
  if (max_age_s > 0.0) {
    const double age_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_offline_info_time_).count();
    if (age_s > max_age_s) {
      return false;
    }
  }
  target_tracking_armed_ = true;
  return true;
}

bool ShengsaiBtNode::hasFreshOfflineInfo(const double max_age_s) const
{
  std::lock_guard<std::mutex> lock(offline_info_mutex_);
  if (!has_offline_info_) {
    return false;
  }
  if (max_age_s <= 0.0) {
    return true;
  }
  const double age_s =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - last_offline_info_time_).count();
  return age_s <= max_age_s;
}

double ShengsaiBtNode::offlineInfoAgeSeconds() const
{
  std::lock_guard<std::mutex> lock(offline_info_mutex_);
  if (!has_offline_info_) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_offline_info_time_).count();
}

void ShengsaiBtNode::populateBlackboard(BT::Blackboard & blackboard) const
{
  blackboard.set("start_x", get_parameter("start_x").as_double());
  blackboard.set("start_y", get_parameter("start_y").as_double());
  blackboard.set("start_yaw_rad", get_parameter("start_yaw_rad").as_double());
  blackboard.set("target_x", get_parameter("target_x").as_double());
  blackboard.set("target_y", get_parameter("target_y").as_double());
  blackboard.set("target_yaw_rad", get_parameter("target_yaw_rad").as_double());
  blackboard.set("system_ready_timeout_s", get_parameter("system_ready_timeout_s").as_double());
  blackboard.set("offline_info_stale_s", get_parameter("offline_info_stale_s").as_double());
  blackboard.set("navigation_timeout_s", get_parameter("navigation_timeout_s").as_double());
  blackboard.set<int>("navigation_retries", static_cast<int>(get_parameter("navigation_retries").as_int()));
  blackboard.set("target_wait_timeout_s", get_parameter("target_wait_timeout_s").as_double());
  blackboard.set("engage_timeout_s", get_parameter("engage_timeout_s").as_double());
}

void ShengsaiBtNode::setSafeMode(const std::string & phase)
{
  combat_active_.store(false);
  aim_allowed_.store(false);
  setPhase(phase);
}

void ShengsaiBtNode::setCombatMode()
{
  if (target_finished_.load()) {
    setSafeMode("TARGET_FINISHED");
    return;
  }
  combat_active_.store(true);
  aim_allowed_.store(true);
  setPhase("ENGAGE");
}

void ShengsaiBtNode::finishCombatAndLatch()
{
  combat_active_.store(false);
  aim_allowed_.store(false);
  target_finished_.store(true);
  setPhase("TARGET_FINISHED");
}

void ShengsaiBtNode::markMissionComplete()
{
  combat_active_.store(false);
  aim_allowed_.store(false);
  setPhase("COMPLETE");
  requestStop();
}

void ShengsaiBtNode::reportFailure(const std::string & reason)
{
  combat_active_.store(false);
  aim_allowed_.store(false);
  if (!failure_reported_.exchange(true)) {
    RCLCPP_ERROR(get_logger(), "Shengsai mission failed: %s", reason.c_str());
  }
  setPhase("FAILED");
  requestStop();
}

void ShengsaiBtNode::requestStop()
{
  if (!stop_requested_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Shengsai mission stopping chassis command.");
  }
  publishStopCommand();
}

void ShengsaiBtNode::offlineInfoCallback(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(offline_info_mutex_);
  is_get_ = msg->is_get;
  has_offline_info_ = true;
  last_offline_info_time_ = std::chrono::steady_clock::now();
  if (target_tracking_armed_ && !msg->is_get) {
    target_finished_.store(true);
  }
}

void ShengsaiBtNode::setPhase(const std::string & phase)
{
  std::lock_guard<std::mutex> lock(phase_mutex_);
  if (phase_ == phase) {
    return;
  }
  phase_ = phase;
  RCLCPP_INFO(get_logger(), "Mission phase: %s", phase_.c_str());
}

void ShengsaiBtNode::publishBehavior()
{
  ros_interfaces::msg::Behavior behavior_msg;
  behavior_msg.desired_stance = combat_active_.load() ? kStanceAttack : kStanceMove;
  behavior_msg.control_mode = kControlAuto;
  behavior_msg.use_gyro_mode = false;
  behavior_msg.gyro_vel = 0.0F;
  behavior_msg.desire_lifter_pos = kLifterTop;

  behavior_msg.ammo_purchase_request = 0;
  behavior_msg.revive_request = 0;
  behavior_msg.remote_revive_request = 0;
  behavior_msg.remote_ammo_request = 0;
  behavior_msg.remote_health_request = 0;

  behavior_msg.use_limited_scan = false;
  behavior_msg.scan_yaw_min = -180.0F;
  behavior_msg.scan_yaw_max = 180.0F;
  behavior_msg.pitch_mode = kPitchBottom;
  behavior_msg.not_aim_enemy = !aim_allowed_.load();
  behavior_msg.use_capacitor = false;

  behavior_msg.tunnel_escape_active = false;
  behavior_msg.tunnel_escape_vx = 0.0F;
  behavior_msg.tunnel_escape_vy = 0.0F;
  behavior_msg.tunnel_escape_wz = 0.0F;
  behavior_msg.tunnel_prepare_active = false;
  behavior_msg.tunnel_ready = false;
  behavior_msg.tunnel_align_active = false;
  behavior_msg.tunnel_align_angle_deg = 0.0F;

  behavior_pub_->publish(behavior_msg);

  if (stop_requested_.load()) {
    publishStopCommand();
  }
}

void ShengsaiBtNode::publishStopCommand()
{
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
}

}  // namespace shengsai_bt
