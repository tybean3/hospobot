#include "nav2_semantic_costmap_plugin/semantic_layer.hpp"
#include <nav2_costmap_2d/costmap_math.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(nav2_semantic_costmap_plugin::SemanticLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_semantic_costmap_plugin
{

SemanticLayer::SemanticLayer()
{
}

SemanticLayer::~SemanticLayer()
{
}

void SemanticLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("topic_name", rclcpp::ParameterValue("/oakd/semantic_objects"));
  declareParameter("timeout_sec", rclcpp::ParameterValue(2.0));

  node->get_parameter(name_ + "." + "topic_name", topic_name_);
  node->get_parameter(name_ + "." + "timeout_sec", timeout_sec_);

  // Load class rules
  std::vector<std::string> classes;
  declareParameter("classes", rclcpp::ParameterValue(std::vector<std::string>()));
  node->get_parameter(name_ + "." + "classes", classes);

  for (const auto& class_name : classes) {
    std::string prefix = name_ + "." + class_name + ".";
    declareParameter(class_name + ".cost_amplitude", rclcpp::ParameterValue(254));
    declareParameter(class_name + ".inflation_radius", rclcpp::ParameterValue(1.0));
    
    int cost;
    double radius;
    node->get_parameter(prefix + "cost_amplitude", cost);
    node->get_parameter(prefix + "inflation_radius", radius);
    
    rules_[class_name] = SemanticRule{cost, radius};
    RCLCPP_INFO(node->get_logger(), "SemanticLayer rule loaded: %s -> cost=%d, radius=%.2f", class_name.c_str(), cost, radius);
  }

  sub_ = node->create_subscription<hospobot_interfaces::msg::SemanticObjectArray>(
    topic_name_, 10,
    std::bind(&SemanticLayer::objectCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(node->get_logger(), "SemanticLayer initialized.");
}

void SemanticLayer::objectCallback(const hospobot_interfaces::msg::SemanticObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_objects_ = msg;
}

void SemanticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_objects_) {
    return;
  }

  auto node = node_.lock();
  if (!node) return;

  auto now = node->now();
  auto msg_time = rclcpp::Time(latest_objects_->header.stamp);
  if ((now - msg_time).seconds() > timeout_sec_) {
    return; // Stale data
  }

  for (const auto& obj : latest_objects_->objects) {
    if (rules_.find(obj.class_name) == rules_.end()) {
      continue; // No rule for this class
    }

    double radius = rules_[obj.class_name].inflation_radius;
    double x = obj.position.x;
    double y = obj.position.y;

    *min_x = std::min(*min_x, x - radius);
    *min_y = std::min(*min_y, y - radius);
    *max_x = std::max(*max_x, x + radius);
    *max_y = std::max(*max_y, y + radius);
  }
}

void SemanticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_objects_) {
    return;
  }

  auto node = node_.lock();
  if (!node) return;

  auto now = node->now();
  auto msg_time = rclcpp::Time(latest_objects_->header.stamp);
  if ((now - msg_time).seconds() > timeout_sec_) {
    return; // Stale data
  }

  for (const auto& obj : latest_objects_->objects) {
    if (rules_.find(obj.class_name) == rules_.end()) {
      continue;
    }

    int base_cost = rules_[obj.class_name].cost_amplitude;
    double radius = rules_[obj.class_name].inflation_radius;

    unsigned int mx, my;
    if (!master_grid.worldToMap(obj.position.x, obj.position.y, mx, my)) {
      continue;
    }

    int radius_cells = static_cast<int>(radius / master_grid.getResolution());

    for (int i = -radius_cells; i <= radius_cells; ++i) {
      for (int j = -radius_cells; j <= radius_cells; ++j) {
        int cx = mx + i;
        int cy = my + j;

        if (cx < min_i || cx >= max_i || cy < min_j || cy >= max_j) {
          continue;
        }
        
        if (cx < 0 || cy < 0 || cx >= (int)master_grid.getSizeInCellsX() || cy >= (int)master_grid.getSizeInCellsY()) {
          continue;
        }

        double distance = std::hypot(i, j) * master_grid.getResolution();
        if (distance <= radius) {
          // Linear decay of cost
          double decay = 1.0 - (distance / radius);
          int added_cost = static_cast<int>(base_cost * decay);
          
          unsigned int index = master_grid.getIndex(cx, cy);
          int current_cost = master_grid.getCost(cx, cy);
          master_grid.setCost(cx, cy, std::max(current_cost, added_cost));
        }
      }
    }
  }
}

void SemanticLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_objects_.reset();
}

}  // namespace nav2_semantic_costmap_plugin
