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
    factory.registerNodeType<shengsai_bt::SetSafeMode>("SetSafeMode");
    factory.registerNodeType<shengsai_bt::WaitForSystemReady>("WaitForSystemReady");
    factory.registerNodeType<shengsai_bt::NavigateToGoal>("NavigateToGoal");
    factory.registerNodeType<shengsai_bt::WaitForTarget>("WaitForTarget");
    factory.registerNodeType<shengsai_bt::EngageUntilTargetLost>("EngageUntilTargetLost");
    factory.registerNodeType<shengsai_bt::CompleteMission>("CompleteMission");
    factory.registerNodeType<shengsai_bt::FailAndHold>("FailAndHold");

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
      tree->tickRoot();
      loop_rate.sleep();
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Behavior tree loop failed: %s", error.what());
  }

  tree->haltTree();
  node->setSafeMode("SHUTDOWN");
  node->requestStop();
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
