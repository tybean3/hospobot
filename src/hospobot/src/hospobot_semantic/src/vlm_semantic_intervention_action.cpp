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

  // VLM Service Client
  vlm_client_ = node_->create_client<hospobot_interfaces::srv::VLMIntervention>("/vlm_intervention");
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
      if (!vlm_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "Waiting for VLM service...");
        return BT::NodeStatus::RUNNING;
      }

      RCLCPP_INFO(node_->get_logger(), "Sending RGB frame to external VLM script...");
      auto request = std::make_shared<hospobot_interfaces::srv::VLMIntervention::Request>();
      request->image = *latest_image_;
      
      future_result_ = vlm_client_->async_send_request(request).share();

      request_sent_ = true;
      latest_image_.reset(); // Clear after sending
    } else {
      if ((now - start_time_).seconds() > 5.0) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for camera image.");
        return BT::NodeStatus::FAILURE;
      }
    }
  } else {
    // Check if the service call is done
    if (future_result_.valid() && future_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto response = future_result_.get();
      if (response->success) {
        RCLCPP_INFO(node_->get_logger(), "Received Immediate Action Bias from VLM. Publishing to local planner...");
        
        geometry_msgs::msg::PoseStamped bias_goal = response->bias_goal;
        bias_goal.header.stamp = node_->now();
        
        bias_pub_->publish(bias_goal);
        setOutput("short_term_bias", bias_goal);
        
        intervention_complete_ = true;
        RCLCPP_INFO(node_->get_logger(), "Semantic Intervention Complete. Robot should be out of deadlock.");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "VLM Service failed: %s", response->message.c_str());
        return BT::NodeStatus::FAILURE;
      }
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
