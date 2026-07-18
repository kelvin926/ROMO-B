#ifndef ROMO_B_NAVIGATION__TRANSIENT_OBSTACLE_LAYER_HPP_
#define ROMO_B_NAVIGATION__TRANSIENT_OBSTACLE_LAYER_HPP_

#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace romo_b_navigation
{

// A PointCloud obstacle layer whose contents are rebuilt from the recent
// observation window on every update. Nav2's standard ObstacleLayer retains a
// marked cell until a later ray explicitly clears that exact cell, which is a
// poor fit for moving people in a corridor.
class TransientObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
};

}  // namespace romo_b_navigation

#endif  // ROMO_B_NAVIGATION__TRANSIENT_OBSTACLE_LAYER_HPP_
