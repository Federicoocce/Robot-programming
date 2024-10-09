#include "world_item.h"
#include <ros/ros.h>
#include <iostream>

using namespace std;
Eigen::Isometry2f WorldItem::globalPose() const {
  if (!parent) return pose_in_parent;
  return parent->globalPose() * pose_in_parent;
}

const GridMap& WorldItem::gridMap() const {
  if (grid_map) return *grid_map;

  WorldItem* p = parent;
  if (p) {
    while (p->parent) {
      if (p->grid_map) return (*p->grid_map);
      p = p->parent;
    }
  }

  throw std::runtime_error("No GridMap available in this branch");
  return *grid_map;
}

void WorldItem::draw(Canvas& canvas, bool show_parent) const {
  Eigen::Vector2f center = grid_map->world2grid(globalPose().translation());
  int radius_px = radius / grid_map->resolution();
  drawCircle(canvas, center, radius_px, 0);

  Eigen::Vector2f x_in_item(radius, 0);
  Eigen::Vector2f x_in_world = globalPose() * x_in_item;
  Eigen::Vector2f x_in_grid = grid_map->world2grid(x_in_world);
  drawLine(canvas, center, x_in_grid, 0);

  if (show_parent == true && parent != nullptr) {
    Eigen::Vector2f parent_in_grid =
        grid_map->world2grid(parent->globalPose().translation());
    drawLine(canvas, center, parent_in_grid, 100);
  }

  for (WorldItem* child : children) {
    child->draw(canvas, show_parent);
  }
}

bool WorldItem::checkCollision() const {
  Eigen::Isometry2f pose = globalPose();
  int radius_px = radius / grid_map->resolution();
  int r2 = radius_px * radius_px;

  Eigen::Vector2f origin_px = grid_map->world2grid(pose.translation());
  //ROS_INFO("Checking collision for robot at (%f, %f), radius: %d pixels", 
           //pose.translation().x(), pose.translation().y(), radius_px);

  for (int r = -radius_px; r <= radius_px; ++r) {
    for (int c = -radius_px; c <= radius_px; ++c) {
      if (r * r + c * c > r2) continue;
      Eigen::Vector2f offset = {c, r};
      Eigen::Vector2f p_px = origin_px + offset;

      if (!grid_map->inside(p_px.cast<int>())) return true;

      if ((*grid_map)(p_px) < 127) return true;
    }
  }

  for (WorldItem* child : children) {
    if (child->checkCollision() == true) return true;
  }

  if (parent && !parent->parent) {
    for (WorldItem* child : parent->children) {
      if (child == this) continue;
      if (checkCollision(*child)) return true;
    }
  }

  return false;
}

bool WorldItem::checkCollision(const WorldItem& other) const {
  if (isAncestor(other)) return false;

  Eigen::Vector2f origin = globalPose().translation();
  Eigen::Vector2f other_origin = other.globalPose().translation();

  // dx, dy => delta
  // distance = squared norm
  // float dx = origin[0] - other_origin[0];
  // float dy = origin[1] - other_origin[1];

  Eigen::Vector2f delta(origin[0] - other_origin[0], origin[1] - other_origin[1]);
  // float distance = sqrt(dx * dx + dy * dy);
  float distance = delta.squaredNorm();

  if (distance < (radius + other.radius)) return true;
  for (WorldItem* child : children) {
    if (child->checkCollision(other)) return true;
  }

  return false;
}

void WorldItem::tick(float time_interval) {
  // do your stuff
  for (WorldItem* child : children) {
    child->tick(time_interval);
  }
}

void World::draw(Canvas& canvas, bool show_parent) const {
  Eigen::Vector2f x_axis(1, 0);
  Eigen::Vector2f y_axis(0, 1);

  Eigen::Vector2f origin = grid_map->world2grid(globalPose().translation());
  Eigen::Vector2f x_axis_on_frame = grid_map->world2grid(globalPose() * x_axis);
  Eigen::Vector2f y_axis_on_frame = grid_map->world2grid(globalPose() * y_axis);

  drawLine(canvas, origin, x_axis_on_frame, 200);
  drawLine(canvas, origin, y_axis_on_frame, 200);

  WorldItem::draw(canvas, show_parent);
}
