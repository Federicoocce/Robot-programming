#include "laser_scan.h"

LaserScan::LaserScan(float range_min, float range_max, float angle_min,
                     float angle_max, int ranges_num) {
  this->range_min = range_min;
  this->range_max = range_max;
  this->angle_min = angle_min;
  this->angle_max = angle_max;
  this->ranges_num = ranges_num;

  // ranges = new float[ranges_num];
  // for (int i = 0; i < ranges_num; ++i) ranges[i] = range_max;
  ranges.resize(ranges_num);
  std::fill(ranges.begin(), ranges.end(), range_max);
}

// LaserScan::~LaserScan() { delete[] ranges; }

void LaserScan::draw(Canvas& canevasso, const GridMap& grid_map,
                     const Eigen::Isometry2f& pose) {
  // change Vec2i => Eigen::Vector2f
  // change Vec2 => Eigen::Vector2f
  // translation => .translation()
  // pose.linear()
  Eigen::Vector2f center_px = grid_map.world2grid(pose.translation());
  float angle_increment = (angle_max - angle_min) / ranges_num;
  Eigen::Isometry2f rotation = pose;
  rotation.translation().setZero();

  for (int i = 0; i < ranges_num; ++i) {
    float beam_angle = angle_min + angle_increment * i;
    Eigen::Vector2f d(cos(beam_angle), sin(beam_angle));
    d = rotation * d;
    Eigen::Vector2f ep = pose.translation() + d * ranges[i];
    Eigen::Vector2f ep_px = grid_map.world2grid(ep);
    drawLine(canevasso, center_px, ep_px, 90);
  }
}