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
    oscillation_timeout_ = 3.0;
  }
  if (!getInput("min_progress_distance", min_progress_distance_)) {
    min_progress_distance_ = 0.3;
  }

  // Subscribe to /odom for high-frequency motion tracking
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&DeadlockDetectorCondition::odomCallback, this, std::placeholders::_1));

  // Also subscribe to /amcl_pose with correct PoseWithCovarianceStamped type
  amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&DeadlockDetectorCondition::amclCallback, this, std::placeholders::_1));
}

DeadlockDetectorCondition::~DeadlockDetectorCondition()
{
}

void DeadlockDetectorCondition::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  StampedPoint pt;
  pt.stamp = rclcpp::Time(msg->header.stamp);
  pt.x = msg->pose.pose.position.x;
  pt.y = msg->pose.pose.position.y;
  pose_history_.push_back(pt);

  auto now = node_->now();
  while (!pose_history_.empty()) {
    if ((now - pose_history_.front().stamp).seconds() > oscillation_timeout_) {
      pose_history_.pop_front();
    } else {
      break;
    }
  }
}

void DeadlockDetectorCondition::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  StampedPoint pt;
  pt.stamp = rclcpp::Time(msg->header.stamp);
  pt.x = msg->pose.pose.position.x;
  pt.y = msg->pose.pose.position.y;
  pose_history_.push_back(pt);

  auto now = node_->now();
  while (!pose_history_.empty()) {
    if ((now - pose_history_.front().stamp).seconds() > oscillation_timeout_) {
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

  auto start_pose = pose_history_.front();
  auto end_pose = pose_history_.back();

  double dx = end_pose.x - start_pose.x;
  double dy = end_pose.y - start_pose.y;
  double distance = std::hypot(dx, dy);

  // If distance is less than minimum progress over at least half the timeout window, trigger deadlock intervention
  auto now = node_->now();
  if ((now - start_pose.stamp).seconds() > (oscillation_timeout_ / 2.0)) {
    if (distance < min_progress_distance_) {
      RCLCPP_WARN(node_->get_logger(), "Deadlock/Oscillation detected! Progress: %.3f m in %.1fs",
                  distance, (now - start_pose.stamp).seconds());
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
