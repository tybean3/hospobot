#include "hospobot_semantic/dense_semantic_layer.hpp"
#include <nav2_costmap_2d/costmap_math.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hospobot_semantic::DenseSemanticLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace hospobot_semantic
{

DenseSemanticLayer::DenseSemanticLayer()
{
}

DenseSemanticLayer::~DenseSemanticLayer()
{
}

void DenseSemanticLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("topic_name", rclcpp::ParameterValue("/semantic_network/dense_costmap"));
  declareParameter("timeout_sec", rclcpp::ParameterValue(2.0));

  node->get_parameter(name_ + "." + "topic_name", topic_name_);
  node->get_parameter(name_ + "." + "timeout_sec", timeout_sec_);

  grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(10).transient_local().reliable(),
    std::bind(&DenseSemanticLayer::gridCallback, this, std::placeholders::_1));

  current_ = true;
  RCLCPP_INFO(node->get_logger(), "DenseSemanticLayer initialized on topic %s", topic_name_.c_str());
}

void DenseSemanticLayer::gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_grid_ = msg;
}

void DenseSemanticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_grid_) {
    return;
  }

  auto node = node_.lock();
  if (!node) return;

  auto now = node->now();
  auto msg_time = rclcpp::Time(latest_grid_->header.stamp);
  if ((now - msg_time).seconds() > timeout_sec_) {
    return; // Stale data
  }

  // Update bounds to cover the entire grid received
  double grid_min_x = latest_grid_->info.origin.position.x;
  double grid_min_y = latest_grid_->info.origin.position.y;
  double grid_max_x = grid_min_x + latest_grid_->info.width * latest_grid_->info.resolution;
  double grid_max_y = grid_min_y + latest_grid_->info.height * latest_grid_->info.resolution;

  *min_x = std::min(*min_x, grid_min_x);
  *min_y = std::min(*min_y, grid_min_y);
  *max_x = std::max(*max_x, grid_max_x);
  *max_y = std::max(*max_y, grid_max_y);
}

void DenseSemanticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_grid_) {
    return;
  }

  auto node = node_.lock();
  if (!node) return;

  auto now = node->now();
  auto msg_time = rclcpp::Time(latest_grid_->header.stamp);
  if ((now - msg_time).seconds() > timeout_sec_) {
    return; // Stale data
  }

  // Map values from semantic grid to master costmap
  for (int i = min_i; i < max_i; ++i) {
    for (int j = min_j; j < max_j; ++j) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // Check if world coordinate is within the semantic grid
      double gx = (wx - latest_grid_->info.origin.position.x) / latest_grid_->info.resolution;
      double gy = (wy - latest_grid_->info.origin.position.y) / latest_grid_->info.resolution;

      int grid_x = static_cast<int>(gx);
      int grid_y = static_cast<int>(gy);

      if (grid_x >= 0 && grid_x < static_cast<int>(latest_grid_->info.width) &&
          grid_y >= 0 && grid_y < static_cast<int>(latest_grid_->info.height)) {
        
        int idx = grid_y * latest_grid_->info.width + grid_x;
        int semantic_cost = latest_grid_->data[idx];

        // Combine costs (taking the maximum)
        if (semantic_cost > 0) {
          int current_cost = master_grid.getCost(i, j);
          int new_cost = std::max(current_cost, semantic_cost);
          master_grid.setCost(i, j, new_cost);
        }
      }
    }
  }
}

void DenseSemanticLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_grid_.reset();
}

}  // namespace hospobot_semantic
