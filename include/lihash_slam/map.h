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

#ifndef INCLUDE_LIHASH_SLAM_MAP_H
#define INCLUDE_LIHASH_SLAM_MAP_H

// G2O
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/types_slam3d.h>

// LiHash-SLAM
#include <lihash_slam/cell.h>
#include <lihash_slam/keyframe.h>

namespace lihash_slam {

// Map
class Map {
 public:
  explicit Map(const double xy_size, const double z_size, const double res);
  virtual ~Map();

  void addKeyframe(lihash_slam::Keyframe* kf);
  void addLoopClosure(const int f1, const int f2, const Eigen::Isometry3d& rel_pose);
  void optimize(const int iters = 1000);
  PointCloud::Ptr getMapPoints();  
  PointCloud::Ptr getLocalMapPoints(const Eigen::Isometry3d& pose, int cells_xy = 2, int cells_z = 1);
  std::vector<Cell*>* getCells();
  std::vector<Keyframe*>* getKeyframes();
 private:
  double voxel_xysize_; // Assume the same size in X and Y dimensions
  double inv_voxel_xysize_;
  double voxel_xysize_half_;
  double voxel_zsize_;
  double inv_voxel_zsize_;
  double voxel_zsize_half_;
  double resolution_;
  pcl::VoxelGrid<Point> vgrid_filter_;

  void clearMapPoints();
  void addMapPoints(const PointCloud::Ptr& pc_in, const Eigen::Isometry3d& pose);

  // Hash table with Cells
  HashMap cells_;
  std::vector<Cell*> cells_vector_; // For a faster iteration

  // Keyframes
  std::vector<Keyframe*> keyframes_;

  // Graph
  g2o::SparseOptimizer optimizer_;
};

}  // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_MAP_H