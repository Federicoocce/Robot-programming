#include <iostream>

#include "grid_map.h"
#include "isometry_2.h"
#include "laser_scan.h"
#include "laser_scanner.h"
#include "world_item.h"


using namespace std;

void printItems(WorldItem** items) {
  while (*items) {
    WorldItem* v = *items;
    cout << "**************************************" << endl
         << "Item: " << v << endl
         << "pose: " << endl
         << v->pose_in_parent << endl
         << "parent: " << v->parent << endl;
    ++items;
  }
}

void drawItems(Canvas& canvas, WorldItem** items) {
  while (*items) {
    WorldItem* v = *items;
    v->draw(canvas, true);
    ++items;
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cout << "usage: " << argv[0] << " <image_file> <resolution>" << endl;
    return -1;
  }
  const char* filename = argv[1];
  Scalar resolution = atof(argv[2]);

  cout << "Running " << argv[0] << " with arguments" << endl
       << "-filename:" << argv[1] << endl
       << "-resolution: " << argv[2] << endl;

  GridMap grid_map(0, 0, 0.1);
  grid_map.loadFromImage(filename, resolution);
  Canvas canvas;
  Vec2 center = grid_map.grid2world(grid_map.origin);
  cerr << "center: " << center[0] << "  " << center[1] << endl;
  cerr << "origin: " << grid_map.origin[0] << "  " << grid_map.origin[1]
       << endl;

  grid_map.draw(canvas);

  WorldItem* items[100];
  memset(items, 0, sizeof(WorldItem*) * 100);

  WorldItem world_object(grid_map);
  items[0] = &world_object;
  WorldItem object_0(world_object, Isometry2(1, 0, 0.5));
  items[1] = &object_0;

  WorldItem object_1(world_object, Isometry2(0, -1, -0.5));
  object_1.radius = 2;
  items[2] = &object_1;

  LaserScan scan;

  LaserScanner scanner(scan, object_1, Isometry2(0, -1, -0.5), 1000);
  scanner.radius = 0.5;
  items[3] = &scanner;

  const Isometry2 forward(0.1, 0, 0);
  const Isometry2 backward(-0.1, 0, 0);
  const Isometry2 left(0, 0, 0.1);
  const Isometry2 right(0, 0, -0.1);

  while (true) {
    scanner.getScan();
    grid_map.draw(canvas);
    drawItems(canvas, items);
    // scan.draw(canvas, grid_map, object_1.globalPose());
    int ret = showCanvas(canvas, 0);
    std::cerr << "Key pressed: " << ret << std::endl;

    Isometry2 motion_iso = Isometry2::Identity();
    switch (ret) {
      case 81:  // left;
        motion_iso = left;
        break;
      case 82:  // up;
        motion_iso = forward;
        break;
      case 83:  // right;
        motion_iso = right;
        break;
      case 84:  // down;
        motion_iso = backward;
        break;
      default:;
    }
    object_1.move(motion_iso);
  }
}
