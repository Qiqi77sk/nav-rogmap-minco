#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include "minco_core/header.hpp"
#include "minco_core/performance/planner_performance_monitor.hpp"
#include "ros_interfaces/msg/sentry_info_offline.hpp"

#include <limits>
#include <optional>

namespace rog_map {
class ROGMapROS;
}  // namespace rog_map

namespace minco_planner {

class Visualizer;
class MincoFsm;

class MincoPlanner : public nav2_core::GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<MincoPlanner>;

  // === Constructor & Lifecycle ===
  MincoPlanner();
  ~MincoPlanner();

  void configure(const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  // === Core Planning Interfaces ===
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal) override;

  bool PlanGlobalPath(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);

  bool ReplanLocal(const geometry_msgs::msg::PoseStamped & current_pose);
  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  // === Callbacks ===
  void safetyTimerCallback();
  void onChassisInfo(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg);

  // === Utility & Helper Functions ===
  bool checkCollision();
  bool checkCollision(const geometry_utils::Trajectory & traj);

  // Accessors for FSM
  bool isTrajSafe() const { return is_traj_safe_.load(); }
  double nowSeconds() const;
  double getTrajectoryRemainTime() const;
  bool isTrajectoryTimeExpired(double now_s) const;
  double getLookaheadDist() const { return lookahead_dist_; }
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose);
  bool consumePendingGoal(geometry_msgs::msg::PoseStamped & goal_out);
  // 检查点门控：只有机器人实际位姿进入 waypoints.reached_radius 才推进路线游标。
  // 由 MincoFsm 在 FOLLOW_TRAJ 中调用；返回 true 表示到达了一个检查点。
  bool tryAdvanceWaypoint(const geometry_msgs::msg::PoseStamped & current_pose);
  void clearPlanVisualization();
  void cancelGoal();
  Eigen::Vector3d getCurrentSpeed() const;
  double getCurrentYawFromOdom() const;
  bool getFreshChassisYawCameraInit(double & yaw) const;

  // Query ESDF distance at the given position.
  double getEsdfDistance(const Eigen::Vector3d & pos) const;

  void publishEscapeCommand(
    const geometry_msgs::msg::PoseStamped & current_pose, const Eigen::Vector2d & escape_vel);

  void clearRecoveryDebugVisualization();

  void publishEmergencyStop(const geometry_msgs::msg::PoseStamped & current_pose);

  traj_opt::Trajectory generateBackupTraj(const Eigen::Matrix3d & start_state);
  std::vector<Eigen::Vector3d> extractLocalPath(const Eigen::Vector3d & cur_pos);

private:
  // === Internal Types ===
  // 一条“终点 -> 中间点序列”的全局路径引导路线。中间点包含 x/y/yaw，
  // yaw 为该检查点要求的到达朝向，只用于本段末段 yaw 规划。
  struct WaypointRoute
  {
    Eigen::Vector2d goal{Eigen::Vector2d::Zero()};
    std::vector<Eigen::Vector3d> points;
  };

  enum class PlanningState
  {
    COLD_START,     // Full replanning with zero initial velocity/acceleration.
    HOT_START,      // Replanning with inherited velocity/acceleration.
    EMERGENCY_STOP  // Immediate backup braking with safety priority.
  };

  // === Utility & Helper Functions ===
  PlanningState determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path);

  void prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state,
    const std::vector<Eigen::Vector3d> & sparse_path);

  void prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose, double t_dur, Eigen::Matrix3d & start_state);

  void PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
    const Eigen::Matrix3d & start_state,
    bool stop_at_local_end,
    PlanningState state,
    bool has_shifted_seed,
    const vec_Vec3f & shifted_waypoints,
    const VecDf & shifted_durations,
    vec_Vec3f & init_ps,
    VecDf & init_ts,
    VecDf & local_vmaxs) const;

  bool validateTrajectory(const traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos);

  bool optimizeYaw(const Eigen::Matrix3d & start_state,
    const traj_opt::Trajectory & pos_traj,
    traj_opt::Trajectory & out_yaw_traj,
    PlanningState state,
    const geometry_msgs::msg::Pose & current_pose,
    double goal_yaw,
    bool free_goal);

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  bool configureRogMap(const nav2_util::LifecycleNode::SharedPtr & node, const std::string & plugin_prefix);

  bool ensureMapAvailable();
  void rebuildModeDependentQueries();
  void configureMincoPerfLogging(const nav2_util::LifecycleNode::SharedPtr & node, const std::string & prefix);

  void initPlannerMode(
    const std::string & planner_mode_param, const std::string & map_frame, const std::string & rog_frame);
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;

  // === Planner-level waypoint routes（检查点门控） ===
  void configureWaypointRoutes(
    const nav2_util::LifecycleNode::SharedPtr & node, const std::string & prefix);
  // 根据最终 goal 选择路线；路线或 goal 变化时把游标清零。
  void updateRouteSelection(const geometry_msgs::msg::PoseStamped & goal);
  // 当前未到达的检查点；没有则为空。
  std::optional<Eigen::Vector3d> activeCheckpoint() const;

  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr backup_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ros_interfaces::msg::SentryInfoOffline>::SharedPtr chassis_info_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // === TF & Costmap & Frames ===
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rog_map::MapQueryInterface> map_;
  std::shared_ptr<rog_map::MapQueryInterface> rog_query_raw_;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_ros_;
  std::string global_frame_, planning_frame_, output_frame_, map_frame_, rog_frame_, name_;
  PlannerModeParams mode_params_;

  // === Configurations & Parameters ===
  double tolerance_;
  bool allow_unknown_;
  bool use_smac_;
  bool use_yaw_opt_{true};
  bool priormap_use_nav2_global_search_{true};
  bool priormap_clip_seed_by_rog_boundary_{true};
  bool exploration_unknown_as_occupied_{true};
  bool exploration_prefer_goal_direction_{true};
  double priormap_rog_boundary_margin_{0.8};
  double priormap_rog_boundary_sample_step_{0.1};
  double exploration_boundary_margin_{0.8};
  double exploration_boundary_sample_step_{0.1};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{-0.2};
  double opt_freq_;
  double lookahead_dist_;
  double traj_goal_tolerance_{0.5};
  bool goal_heading_enable_{false};
  double goal_heading_brake_trigger_dist_{1.0};
  double goal_heading_blend_width_{0.5};
  double yaw_goal_tolerance_{0.15};
  // 全局路径检查点（门控：实际位姿进入 reached_radius 才推进到下一个点）
  std::vector<WaypointRoute> waypoint_routes_;
  bool waypoint_enable_{false};
  double waypoint_match_radius_{0.6};
  double waypoint_reached_radius_{0.30};
  int active_route_index_{-1};
  bool has_route_goal_{false};
  Eigen::Vector2d route_goal_xy_{Eigen::Vector2d::Zero()};
  size_t route_cursor_{0};
  // 当前全局路径只规划到检查点时，checkGoalReached 必须改用最终 Nav2 目标
  geometry_msgs::msg::PoseStamped route_final_goal_;
  bool has_route_final_goal_{false};
  MincoOptimizer::Config minco_config;
  RecoverServer::Config recovery_server_config_{};
  PlannerPerformanceMonitor planner_perf_monitor_;

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  std::unique_ptr<Astar> astar_planner_;
  std::unique_ptr<minco_planner::smac::SmacPlanner2DSimple> smac_planner_;
  std::unique_ptr<MincoOptimizer> minco_optimizer_;
  std::unique_ptr<traj_opt::BackupTrajOpt> backup_opt_;
  std::unique_ptr<traj_opt::YawTrajOpt> yaw_opt_;
  std::unique_ptr<PlannerModeContext> mode_context_;
  std::unique_ptr<GlobalPathSearcher> global_path_searcher_;
  std::unique_ptr<LocalPathProcessor> local_path_processor_;
  std::unique_ptr<TrajectorySafetyChecker> safety_checker_;
  std::unique_ptr<MincoFsm> fsm_;
  SimpleCorridorGenerator::Ptr corridor_gen_;
  RecoverServer::Ptr recovery_server_;
  Ptr planner_handle_;

  // === State Variables & Caches ===
  uint32_t opt_trajectory_id_{0};
  uint32_t backup_trajectory_id_{0};
  geometry_utils::Trajectory last_traj_;
  geometry_utils::Trajectory last_yaw_traj_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  nav_msgs::msg::Odometry latest_odom_;
  double latest_chassis_yaw_camerainit_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point latest_chassis_yaw_rx_{};
  geometry_msgs::msg::PoseStamped pending_goal_;

  bool has_last_traj_ = false;
  bool has_last_yaw_traj_ = false;
  bool has_pending_goal_{false};
  bool has_latest_odom_{false};
  std::atomic_bool is_traj_safe_{true};

  std::mutex path_mutex_;
  std::mutex goal_mutex_;
  std::mutex perf_mutex_;
  mutable std::mutex odom_mutex_;
  mutable std::mutex chassis_yaw_mutex_;
  mutable std::mutex mutex_;

  std::unique_ptr<Visualizer> visualizer_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
  double last_global_search_time_ms_{std::numeric_limits<double>::quiet_NaN()};
  bool has_fresh_global_search_time_{false};
  long long last_minco_perf_stamp_ns_{0};
  std::string last_validation_failure_reason_{"KINEMATIC_VIOLATION"};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
