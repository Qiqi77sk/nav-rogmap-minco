#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include "shengsai_bt/bt_nodes.hpp"
#include "shengsai_bt/shengsai_bt_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<shengsai_bt::ShengsaiBtNode> node;
  std::unique_ptr<BT::Tree> tree;
  try {
    node = std::make_shared<shengsai_bt::ShengsaiBtNode>();
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", std::static_pointer_cast<rclcpp::Node>(node));
    node->populateBlackboard(*blackboard);

    BT::BehaviorTreeFactory factory;
    // Only the active three-node mission is registered. The legacy nodes remain
    // in bt_nodes.cpp for history but are no longer part of the running tree.
    factory.registerNodeType<shengsai_bt::NavigateToGoal>("NavigateToGoal");
    factory.registerNodeType<shengsai_bt::EngageAtTarget>("EngageAtTarget");

    const std::string tree_path =
      ament_index_cpp::get_package_share_directory("shengsai_bt") + "/tree/shengsai_mission.xml";
    tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(tree_path, blackboard));
    RCLCPP_INFO(node->get_logger(), "Loaded shengsai mission tree: %s", tree_path.c_str());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "shengsai_bt initialization failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  const double configured_hz = node->get_parameter("bt_tick_hz").as_double();
  const double tick_hz = std::clamp(configured_hz, 1.0, 100.0);
  rclcpp::Rate loop_rate(tick_hz);
  RCLCPP_INFO(node->get_logger(), "Shengsai mission started at %.1f Hz", tick_hz);

  try {
    while (rclcpp::ok()) {
      const auto status = tree->tickRoot();
      if (status == BT::NodeStatus::SUCCESS) {
        node->markMissionComplete();
        node->setZeroHold(true);
        RCLCPP_INFO(node->get_logger(), "Mission completed; holding terminal safe state.");
        break;
      }
      if (status == BT::NodeStatus::FAILURE) {
        node->reportFailure("mission sequence failed");
        node->setZeroHold(true);
        break;
      }
      loop_rate.sleep();
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Behavior tree loop failed: %s", error.what());
    tree->haltTree();
    node->reportFailure(std::string("behavior tree exception: ") + error.what());
    node->setZeroHold(true);
  }

  // The mission is latched after SUCCESS or FAILURE. Keep the ROS node and the
  // 10 Hz safety publisher alive instead of tearing down the executor.
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  tree->haltTree();
  node->requestStop();
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
