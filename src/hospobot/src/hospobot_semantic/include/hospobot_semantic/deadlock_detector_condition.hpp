#ifndef HOSPOBOT_SEMANTIC__DEADLOCK_DETECTOR_CONDITION_HPP_
#define HOSPOBOT_SEMANTIC__DEADLOCK_DETECTOR_CONDITION_HPP_

#include <string>
#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace hospobot_semantic
{

struct StampedPoint
{
  rclcpp::Time stamp;
  double x;
  double y;
};

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
      BT::InputPort<double>("oscillation_timeout", 3.0, "Time in seconds to check for oscillation/deadlock"),
      BT::InputPort<double>("min_progress_distance", 0.3, "Minimum distance to consider as progress")
    };
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  std::deque<StampedPoint> pose_history_;
  double oscillation_timeout_;
  double min_progress_distance_;
  std::mutex pose_mutex_;
};

}  // namespace hospobot_semantic

#endif  // HOSPOBOT_SEMANTIC__DEADLOCK_DETECTOR_CONDITION_HPP_
