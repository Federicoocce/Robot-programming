#pragma once
#include "laser_scan.h"
#include "world_item.h"

struct LaserScanner : public WorldItem {
  LaserScan& scan;
  float period = 0;  // 1 / frequency;
  float counter = 0;
  bool scan_ready = false;

  LaserScanner(LaserScan& scn, WorldItem& par, const Eigen::Isometry2f& pos,
               float frequency);

  void draw(Canvas& canvas, bool show_parent) const;

  void tick(float dt) override;

  void getScan();
};