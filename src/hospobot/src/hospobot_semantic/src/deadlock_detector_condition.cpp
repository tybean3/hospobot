#include "hospobot_semantic/deadlock_detector_condition.hpp"
#include <cmath>

namespace hospobot_semantic
{

DeadlockDetectorCondition::DeadlockDetectorCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  if (!getInput("oscillation_timeout", oscillation_timeout_)) {
    oscillation_timeout_ = 5.0;
  }
  if (!getInput("min_progress_distance", min_progress_distance_)) {
    min_progress_distance_ = 0.5;
  }

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/amcl_pose", 10,
    std::bind(&DeadlockDetectorCondition::poseCallback, this, std::placeholders::_1));
}

DeadlockDetectorCondition::~DeadlockDetectorCondition()
{
}

void DeadlockDetectorCondition::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  pose_history_.push_back(*msg);
  
  auto now = node_->now();
  // Remove old poses
  while (!pose_history_.empty()) {
    auto pose_time = rclcpp::Time(pose_history_.front().header.stamp);
    if ((now - pose_time).seconds() > oscillation_timeout_) {
      pose_history_.pop_front();
    } else {
      break;
    }
  }
}

BT::NodeStatus DeadlockDetectorCondition::tick()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  if (pose_history_.empty()) {
    return BT::NodeStatus::FAILURE; // Not enough data
  }

  // Calculate distance traveled within the timeout window
  auto start_pose = pose_history_.front();
  auto end_pose = pose_history_.back();

  double dx = end_pose.pose.position.x - start_pose.pose.position.x;
  double dy = end_pose.pose.position.y - start_pose.pose.position.y;
  double distance = std::hypot(dx, dy);

  // If distance is less than minimum progress, we might be deadlocked or oscillating
  // Ensure we have at least half of the timeout window of data
  auto start_time = rclcpp::Time(start_pose.header.stamp);
  auto end_time = rclcpp::Time(end_pose.header.stamp);
  if ((end_time - start_time).seconds() > (oscillation_timeout_ / 2.0)) {
    if (distance < min_progress_distance_) {
      RCLCPP_WARN(node_->get_logger(), "Deadlock/Oscillation detected! Progress: %f m", distance);
      return BT::NodeStatus::SUCCESS; // Trigger semantic intervention
    }
  }

  return BT::NodeStatus::FAILURE; // Doing fine, no deadlock
}

}  // namespace hospobot_semantic

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hospobot_semantic::DeadlockDetectorCondition>("DeadlockDetector");
}
