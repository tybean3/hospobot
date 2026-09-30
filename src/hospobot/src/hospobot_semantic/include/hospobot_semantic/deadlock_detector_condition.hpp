#ifndef HOSPOBOT_SEMANTIC__DEADLOCK_DETECTOR_CONDITION_HPP_
#define HOSPOBOT_SEMANTIC__DEADLOCK_DETECTOR_CONDITION_HPP_

#include <string>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace hospobot_semantic
{

class DeadlockDetectorCondition : public BT::ConditionNode
{
public:
  DeadlockDetectorCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  DeadlockDetectorCondition() = delete;

  ~DeadlockDetectorCondition() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("oscillation_timeout", 5.0, "Time in seconds to check for oscillation/deadlock"),
      BT::InputPort<double>("min_progress_distance", 0.5, "Minimum distance to consider as progress")
    };
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  std::deque<geometry_msgs::msg::PoseStamped> pose_history_;
  double oscillation_timeout_;
  double min_progress_distance_;
  std::mutex pose_mutex_;
};

}  // namespace hospobot_semantic

#endif  // HOSPOBOT_SEMANTIC__DEADLOCK_DETECTOR_CONDITION_HPP_
