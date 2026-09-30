#ifndef HOSPOBOT_SEMANTIC__VLM_SEMANTIC_INTERVENTION_ACTION_HPP_
#define HOSPOBOT_SEMANTIC__VLM_SEMANTIC_INTERVENTION_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "hospobot_interfaces/srv/vlm_intervention.hpp"

namespace hospobot_semantic
{

class VLMSemanticInterventionAction : public BT::StatefulActionNode
{
public:
  VLMSemanticInterventionAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  ~VLMSemanticInterventionAction() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("camera_topic", "/oakd/rgb/image_raw", "Camera topic for VLM"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("short_term_bias", "Short-term goal for local planner")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr bias_pub_;
  rclcpp::Client<hospobot_interfaces::srv::VLMIntervention>::SharedPtr vlm_client_;
  std::shared_future<std::shared_ptr<hospobot_interfaces::srv::VLMIntervention_Response>> future_result_;
  
  sensor_msgs::msg::Image::SharedPtr latest_image_;
  std::mutex image_mutex_;
  
  bool request_sent_;
  bool intervention_complete_;
  rclcpp::Time start_time_;
};

}  // namespace hospobot_semantic

#endif  // HOSPOBOT_SEMANTIC__VLM_SEMANTIC_INTERVENTION_ACTION_HPP_
