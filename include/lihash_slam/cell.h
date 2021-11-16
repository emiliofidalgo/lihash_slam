/*
* This file is part of lihash_slam.
*
* Copyright (C) 2021 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
*
* lihash_slam is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* lihash_slam is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with lihash_slam. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCLUDE_LIHASH_SLAM_CELL_H
#define INCLUDE_LIHASH_SLAM_CELL_H

#include <unordered_map>
#include <vector>

// PCL
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

// LiHash-SLAM
#include <lihash_slam/defs.h>

namespace lihash_slam {

// Cell
class Cell {
 public:
  Cell();
  explicit Cell(const Eigen::Vector3d& pose);
  explicit Cell(const PointCloud::Ptr& pc, const Eigen::Vector3d& pose);
  virtual ~Cell();
  
  void addPoint(const Point& p);
  void addPoints(const PointCloud::Ptr& pc);
  void clear();
  void filter(pcl::VoxelGrid<Point>& filt);
  bool modified();  
  PointCloud::Ptr getPoints();
  Eigen::Vector3d getPose();

 private:
  PointCloud::Ptr points_;
  Eigen::Vector3d pose_;
  bool modified_;
};

// HashKey
struct HashKey {
  HashKey() {}
  explicit HashKey(int x_, int y_, int z_) : 
    x(x_),
    y(y_),
    z(z_) {}

  bool operator==(const HashKey& other) const {
    if (this->x == other.x && 
        this->y == other.y &&
        this->z == other.z)
        return true;
    else
      return false;
  }

  struct HashFunc {
    size_t operator()(const HashKey& k) const {
      size_t h1 = std::hash<int>()(k.x);
      size_t h2 = std::hash<int>()(k.y);
      size_t h3 = std::hash<int>()(k.z);
      return (h1 ^ (h2 << 1)) ^ (h3 << 2);
    }
  };

  int x;
  int y;
  int z;
};

// Defining a map of cells referred using its corresponding HashKey
typedef std::unordered_map<HashKey, Cell*, HashKey::HashFunc> HashMap;

}  // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_CELL_H