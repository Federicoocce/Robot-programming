#pragma once
#include "grid_map.h"
#include <ros/ros.h>
// #include "isometry_2.h"

class WorldItem {
 public:

  static constexpr int CHILDREN_MAX_NUM = 10;
  Eigen::Isometry2f pose_in_parent;
  WorldItem* parent;
  const GridMap* grid_map = 0;
  float radius = 1;
  std::set<WorldItem*> children;

 protected:
  WorldItem(const GridMap* g, WorldItem* p,
            const Eigen::Isometry2f& iso = Eigen::Isometry2f::Identity())
      : grid_map(g), parent(p), pose_in_parent(iso) {
    
    if (parent) {
      parent->children.insert(this);
    }
  }

 public:
  WorldItem(const GridMap& g) : WorldItem(&g, 0) {}

  WorldItem(WorldItem& p, const Eigen::Isometry2f& iso = Eigen::Isometry2f::Identity())
      : WorldItem(p.grid_map, &p, iso) {}

  ~WorldItem() {
    if (parent) {
      parent->children.erase(this);
    }
  }

  Eigen::Isometry2f globalPose() const;

  const GridMap& gridMap() const;

  inline bool isAncestor(const WorldItem& other) const {
    if (!parent) return false;
    if (parent == &other) return true;
    return parent->isAncestor(other);
  }


  bool checkCollision() const;
  bool checkCollision(const WorldItem& other) const;

inline bool move(const Eigen::Isometry2f& iso) {
    Eigen::Isometry2f restored_pose_in_parent = pose_in_parent;
    pose_in_parent = pose_in_parent * iso;
    if (checkCollision()) {
        ROS_INFO("Collision detected, movement cancelled");
        pose_in_parent = restored_pose_in_parent;
        return false;
    }
    ROS_INFO("Movement successful");
    return true;
}

  virtual void draw(Canvas& canvas, bool show_parent) const;

  virtual void tick(float time_interval);
};

class World : public WorldItem {
 public:
  World(const GridMap& g) : WorldItem(g) {}

  void draw(Canvas& canvas, bool show_parent) const override;
};

class UnicyclePlatform : public WorldItem {
 public:
  float tvel = 0, rvel = 0;
  UnicyclePlatform(WorldItem& p, const Eigen::Isometry2f& iso = Eigen::Isometry2f::Identity())
      : WorldItem(p.grid_map, &p, iso) {}

  inline void tick(float time_interval) override {
    // Eigen::Isometry2f delta_state(tvel * time_interval, 0, rvel * time_interval);
    Eigen::Isometry2f delta_state;
    delta_state.translation() << tvel * time_interval, 0;
    delta_state.linear() = Eigen::Rotation2Df(rvel * time_interval).matrix();
    move(delta_state);
    WorldItem::tick(time_interval);
  }

  void draw(Canvas& canvas, bool show_parent) const override {
    WorldItem::draw(canvas, show_parent);
  }
};
