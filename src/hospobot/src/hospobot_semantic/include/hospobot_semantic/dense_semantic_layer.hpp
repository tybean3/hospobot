#ifndef HOSPOBOT_SEMANTIC__DENSE_SEMANTIC_LAYER_HPP_
#define HOSPOBOT_SEMANTIC__DENSE_SEMANTIC_LAYER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mutex>

namespace hospobot_semantic
{

class DenseSemanticLayer : public nav2_costmap_2d::Layer
{
public:
  DenseSemanticLayer();
  virtual ~DenseSemanticLayer();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  virtual void reset() override;
  virtual bool isClearable() override { return true; }

private:
  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
  std::mutex data_mutex_;
  std::string topic_name_;
  double timeout_sec_;
};

}  // namespace hospobot_semantic

#endif  // HOSPOBOT_SEMANTIC__DENSE_SEMANTIC_LAYER_HPP_
