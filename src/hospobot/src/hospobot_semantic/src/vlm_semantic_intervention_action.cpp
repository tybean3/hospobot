#include "hospobot_semantic/vlm_semantic_intervention_action.hpp"

namespace hospobot_semantic
{

VLMSemanticInterventionAction::VLMSemanticInterventionAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  request_sent_(false),
  intervention_complete_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string camera_topic;
  if (!getInput("camera_topic", camera_topic)) {
    camera_topic = "/oakd/rgb/image_raw";
  }

  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    camera_topic, 1,
    std::bind(&VLMSemanticInterventionAction::imageCallback, this, std::placeholders::_1));

  // Publisher to send the short-term bias to the local planner
  bias_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/semantic_bias_goal", 10);
}

VLMSemanticInterventionAction::~VLMSemanticInterventionAction()
{
}

void VLMSemanticInterventionAction::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_image_ = msg;
}

BT::NodeStatus VLMSemanticInterventionAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "Starting VLM Semantic Intervention...");
  
  // Pause the standard navigation controller (e.g., by canceling the current goal or ignoring standard cmd_vel)
  // Here we assume the behavior tree logic effectively pauses standard navigation when this action is running
  
  request_sent_ = false;
  intervention_complete_ = false;
  latest_image_.reset();
  start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VLMSemanticInterventionAction::onRunning()
{
  if (intervention_complete_) {
    return BT::NodeStatus::SUCCESS;
  }

  auto now = node_->now();

  if (!request_sent_) {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (latest_image_) {
      RCLCPP_INFO(node_->get_logger(), "Sending RGB frame to external VLM script...");
      // TODO: Make actual service call to VLM here using latest_image_
      // Example:
      // auto request = std::make_shared<hospobot_interfaces::srv::VLMIntervention::Request>();
      // request->image = *latest_image_;
      // vlm_client_->async_send_request(request, [this](...) { ... process response ... });

      request_sent_ = true;
      latest_image_.reset(); // Clear after sending
    } else {
      if ((now - start_time_).seconds() > 5.0) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for camera image.");
        return BT::NodeStatus::FAILURE;
      }
    }
  } else {
    // Simulated VLM Response Handling (replace with actual service callback logic)
    // Assume we received a response after 2 seconds
    if ((now - start_time_).seconds() > 2.0) {
      RCLCPP_INFO(node_->get_logger(), "Received Immediate Action Bias from VLM. Publishing to local planner...");
      
      geometry_msgs::msg::PoseStamped bias_goal;
      bias_goal.header.stamp = node_->now();
      bias_goal.header.frame_id = "base_link"; // Bias is typically relative to the robot
      bias_goal.pose.position.x = 1.0; // Example: Move 1 meter forward
      bias_goal.pose.position.y = 0.0;
      bias_goal.pose.orientation.w = 1.0;

      bias_pub_->publish(bias_goal);
      setOutput("short_term_bias", bias_goal);
      
      intervention_complete_ = true;
      RCLCPP_INFO(node_->get_logger(), "Semantic Intervention Complete. Robot should be out of deadlock.");
    }
  }

  return BT::NodeStatus::RUNNING;
}

void VLMSemanticInterventionAction::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "VLM Semantic Intervention Halted.");
  request_sent_ = false;
  intervention_complete_ = false;
}

}  // namespace hospobot_semantic

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hospobot_semantic::VLMSemanticInterventionAction>("VLMSemanticIntervention");
}
