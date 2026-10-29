#ifndef MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
#define MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_

#include "minco_core/header.hpp"
#include "smac_search/smac_planner_2d_simple.hpp"

namespace minco_planner {

class PlannerModeContext;

class GlobalPathSearcher
{
public:
  void configure(std::shared_ptr<tf2_ros::Buffer> tf,
    Astar * astar,
    smac::SmacPlanner2DSimple * smac,
    bool use_smac,
    bool allow_unknown,
    double tolerance,
    rclcpp::Logger logger);

  void setQuery(const std::shared_ptr<rog_map::MapQueryInterface> & global_query);

  // 单段全局路径搜索。
  // relax_goal_esdf=true 时，只对“目标格”放宽实时 ESDF 的 collision_dist 判定，
  // 用于中间检查点（路点是引导点，不是最终避障目标）。
  bool plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path,
    bool relax_goal_esdf = false);

  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const PlannerModeContext & mode_context,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

private:
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;

  bool planPriorMap(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path,
    bool relax_goal_esdf);

  bool planExploration(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const PlannerModeContext & mode_context,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path,
    bool relax_goal_esdf);

  bool makePlanOnQuery(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    const std::string & output_frame,
    const std::string & failure_source,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan,
    std::vector<geometry_msgs::msg::PoseStamped> & latest_global_path,
    bool relax_goal_esdf);

  std::shared_ptr<tf2_ros::Buffer> tf_;
  Astar * astar_{nullptr};
  smac::SmacPlanner2DSimple * smac_{nullptr};
  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  bool use_smac_{false};
  bool allow_unknown_{true};
  double tolerance_{0.5};
  rclcpp::Logger logger_{rclcpp::get_logger("GlobalPathSearcher")};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GLOBAL_PATH_SEARCHER_HPP_
