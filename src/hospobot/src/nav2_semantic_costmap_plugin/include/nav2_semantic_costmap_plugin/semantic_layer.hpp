#ifndef NAV2_SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_LAYER_HPP_
#define NAV2_SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_math.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <hospobot_interfaces/msg/semantic_object_array.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace nav2_semantic_costmap_plugin
{

class SemanticLayer : public nav2_costmap_2d::Layer
{
public:
  SemanticLayer();
  virtual ~SemanticLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  virtual void reset() override;
  virtual bool isClearable() override { return false; }

protected:
  void objectCallback(const hospobot_interfaces::msg::SemanticObjectArray::SharedPtr msg);

  rclcpp::Subscription<hospobot_interfaces::msg::SemanticObjectArray>::SharedPtr sub_;
  hospobot_interfaces::msg::SemanticObjectArray::SharedPtr latest_objects_;
  std::mutex data_mutex_;

  // Parameters
  std::string topic_name_;
  double timeout_sec_;

  struct SemanticRule {
    int cost_amplitude;
    double inflation_radius;
  };
  std::unordered_map<std::string, SemanticRule> rules_;
};

}  // namespace nav2_semantic_costmap_plugin

#endif  // NAV2_SEMANTIC_COSTMAP_PLUGIN__SEMANTIC_LAYER_HPP_
