#include "hospobot_semantic/dense_semantic_layer.hpp"
#include <nav2_costmap_2d/costmap_math.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <algorithm>
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
    topic_name_, rclcpp::QoS(1).reliable(),
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

  if (latest_grid_->header.stamp.sec != 0) {
    auto now = node->now();
    auto msg_time = rclcpp::Time(latest_grid_->header.stamp);
    if (std::abs((now - msg_time).seconds()) > timeout_sec_) {
      return; // Stale data
    }
  }

  geometry_msgs::msg::TransformStamped transform_to_global;
  try {
    transform_to_global = tf_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), latest_grid_->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                         "DenseSemanticLayer TF Error: %s", ex.what());
    return;
  }

  tf2::Transform tf2_to_global(
    tf2::Quaternion(
      transform_to_global.transform.rotation.x,
      transform_to_global.transform.rotation.y,
      transform_to_global.transform.rotation.z,
      transform_to_global.transform.rotation.w),
    tf2::Vector3(
      transform_to_global.transform.translation.x,
      transform_to_global.transform.translation.y,
      transform_to_global.transform.translation.z)
  );

  double w = latest_grid_->info.width * latest_grid_->info.resolution;
  double h = latest_grid_->info.height * latest_grid_->info.resolution;
  double ox = latest_grid_->info.origin.position.x;
  double oy = latest_grid_->info.origin.position.y;

  tf2::Vector3 p1(ox, oy, 0);
  tf2::Vector3 p2(ox + w, oy, 0);
  tf2::Vector3 p3(ox, oy + h, 0);
  tf2::Vector3 p4(ox + w, oy + h, 0);

  p1 = tf2_to_global * p1;
  p2 = tf2_to_global * p2;
  p3 = tf2_to_global * p3;
  p4 = tf2_to_global * p4;

  *min_x = std::min({*min_x, p1.x(), p2.x(), p3.x(), p4.x()});
  *min_y = std::min({*min_y, p1.y(), p2.y(), p3.y(), p4.y()});
  *max_x = std::max({*max_x, p1.x(), p2.x(), p3.x(), p4.x()});
  *max_y = std::max({*max_y, p1.y(), p2.y(), p3.y(), p4.y()});
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

  if (latest_grid_->header.stamp.sec != 0) {
    auto now = node->now();
    auto msg_time = rclcpp::Time(latest_grid_->header.stamp);
    if (std::abs((now - msg_time).seconds()) > timeout_sec_) {
      return; // Stale data
    }
  }

  geometry_msgs::msg::TransformStamped transform_to_sensor;
  try {
    transform_to_sensor = tf_->lookupTransform(
      latest_grid_->header.frame_id, layered_costmap_->getGlobalFrameID(), tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return;
  }

  tf2::Transform tf2_to_sensor(
    tf2::Quaternion(
      transform_to_sensor.transform.rotation.x,
      transform_to_sensor.transform.rotation.y,
      transform_to_sensor.transform.rotation.z,
      transform_to_sensor.transform.rotation.w),
    tf2::Vector3(
      transform_to_sensor.transform.translation.x,
      transform_to_sensor.transform.translation.y,
      transform_to_sensor.transform.translation.z)
  );

  // Optimized update loop to avoid matrix multiplication in the inner loop
  double master_res = master_grid.getResolution();
  tf2::Vector3 dx_sensor = tf2_to_sensor.getBasis() * tf2::Vector3(master_res, 0.0, 0.0);
  tf2::Vector3 dy_sensor = tf2_to_sensor.getBasis() * tf2::Vector3(0.0, master_res, 0.0);

  double origin_x, origin_y;
  master_grid.mapToWorld(min_i, min_j, origin_x, origin_y);
  tf2::Vector3 start_sensor_pt = tf2_to_sensor * tf2::Vector3(origin_x, origin_y, 0.0);

  double inv_res = 1.0 / latest_grid_->info.resolution;
  double grid_ox = latest_grid_->info.origin.position.x;
  double grid_oy = latest_grid_->info.origin.position.y;
  int grid_w = latest_grid_->info.width;
  int grid_h = latest_grid_->info.height;

  for (int j = min_j; j < max_j; ++j) {
    tf2::Vector3 sensor_pt = start_sensor_pt + dy_sensor * (j - min_j);
    for (int i = min_i; i < max_i; ++i) {
      double gx = (sensor_pt.x() - grid_ox) * inv_res;
      double gy = (sensor_pt.y() - grid_oy) * inv_res;

      if (gx >= 0.0 && gy >= 0.0) {
        int grid_x = static_cast<int>(gx);
        int grid_y = static_cast<int>(gy);

        if (grid_x < grid_w && grid_y < grid_h) {
          int idx = grid_y * grid_w + grid_x;
          int semantic_cost = latest_grid_->data[idx];

          if (semantic_cost > 0) {
            int mapped_cost = (semantic_cost >= 100) ? nav2_costmap_2d::LETHAL_OBSTACLE : static_cast<int>((semantic_cost / 99.0) * 252.0);
            int current_cost = master_grid.getCost(i, j);
            if (mapped_cost > current_cost) {
              master_grid.setCost(i, j, mapped_cost);
            }
          }
        }
      }
      sensor_pt += dx_sensor;
    }
  }

}

void DenseSemanticLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_grid_.reset();
}

}  // namespace hospobot_semantic
