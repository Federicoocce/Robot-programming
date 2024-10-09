#include "laser_scanner.h"

LaserScanner::LaserScanner(LaserScan& scn, WorldItem& par, const Eigen::Isometry2f& pos,
                           float frequency)
    : WorldItem(par, pos), scan(scn) {
  period = 1 / frequency;
}

void LaserScanner::draw(Canvas& canvas, bool show_parent) const {
  scan.draw(canvas, *grid_map, globalPose());
}

void LaserScanner::getScan() {
  Eigen::Isometry2f gp = globalPose();
  Eigen::Isometry2f rotation = gp;
  rotation.translation().setZero();
  float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges_num;

  for (int i = 0; i < scan.ranges_num; ++i) {
    float beam_angle = scan.angle_min + angle_increment * i;
    Eigen::Vector2f d(cos(beam_angle), sin(beam_angle));
    d = rotation * d;
    scan.ranges[i] = grid_map->scanRay(gp.translation(), d, scan.range_max);
  }
}

void LaserScanner::tick(float dt) {
  WorldItem::tick(dt);
  counter += dt;
  scan_ready = false;
  if (counter > period) {
    counter -= period;
    scan_ready = true;
    getScan();
  }
}