#include "romo_b_navigation/transient_obstacle_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace romo_b_navigation
{

void TransientObstacleLayer::updateBounds(
  const double robot_x, const double robot_y, const double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  // Keep the rolling layer aligned before clearing it. resetMaps() affects
  // only this dynamic layer; the static map remains owned by StaticLayer.
  if (rolling_window_) {
    updateOrigin(
      robot_x - getSizeInMetersX() * 0.5,
      robot_y - getSizeInMetersY() * 0.5);
  }
  resetMaps();

  // The prior obstacle may be anywhere in the layer. Expand the update bounds
  // so its old master-grid cost is overwritten, then mark only observations
  // still present in the configured short persistence window.
  touch(getOriginX(), getOriginY(), min_x, min_y, max_x, max_y);
  touch(
    getOriginX() + getSizeInMetersX(),
    getOriginY() + getSizeInMetersY(),
    min_x, min_y, max_x, max_y);
  nav2_costmap_2d::ObstacleLayer::updateBounds(
    robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

}  // namespace romo_b_navigation

PLUGINLIB_EXPORT_CLASS(
  romo_b_navigation::TransientObstacleLayer,
  nav2_costmap_2d::Layer)
