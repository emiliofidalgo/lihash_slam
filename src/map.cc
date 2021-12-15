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

#include <lihash_slam/map.h>

#include <random>

G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace lihash_slam {

Map::Map(const double xy_size, const double z_size, const double res) :
  voxel_xysize_(xy_size),
  inv_voxel_xysize_(1.0 / xy_size),
  voxel_xysize_half_(xy_size / 2.0),
  voxel_zsize_(z_size),
  inv_voxel_zsize_(1.0 / z_size),
  voxel_zsize_half_(z_size / 2.0),
  resolution_(res) {

    // VoxelGrid filter
	  vgrid_filter_.setLeafSize(resolution_, resolution_, resolution_);

    // Setting up g2o
    optimizer_.setVerbose(false);

    // Setting the solver
    g2o::OptimizationAlgorithmFactory* solverFactory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solverProperty;
    g2o::OptimizationAlgorithm* solver = solverFactory->construct("lm_var_cholmod", solverProperty);
    optimizer_.setAlgorithm(solver);

    if(!optimizer_.solver()) {
      ROS_ERROR("Error allocating LM Cholmod solver!");
      ROS_ERROR("Available solvers: ");
      solverFactory->listSolvers(std::cout);
    }
}

Map::~Map() {
  // Delete memory allocated dynamically
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    delete cells_vector_[i];
  }

  for (size_t i = 0; i < keyframes_.size(); i++) {
    delete keyframes_[i];
  }
}

void Map::addKeyframe(lihash_slam::Keyframe* kf) {

  // Computing the global pose of the Keyframe
  if (keyframes_.size() > 0) {
    kf->pose = keyframes_[keyframes_.size() - 1]->pose * kf->pose;
  }  
  
  // Adding the keyframe to the map
  keyframes_.push_back(kf);  
  
  // Adding the node to the graph
  g2o::VertexSE3* v(new g2o::VertexSE3());
  v->setId(kf->id);
  v->setEstimate(kf->pose);
  kf->node = v;
  optimizer_.addVertex(v);

  // Checking if this is the first node
  bool first_kf = keyframes_.size() == 1;
  if (first_kf) {
    v->setFixed(true);
  } else {
    v->setFixed(false);

    // Adding an edge with the previous node
    lihash_slam::Keyframe* prev_kf = keyframes_[keyframes_.size() - 2];
    Eigen::Isometry3d rel_pose = prev_kf->pose.inverse() * kf->pose;

    // Compute information matrix
    Eigen::MatrixXd im = infcalc_.calcInfMatrix(prev_kf->points, kf->points, rel_pose);

    g2o::EdgeSE3* e(new g2o::EdgeSE3());
    e->setMeasurement(rel_pose);
    e->setInformation(im);
    e->vertices()[0] = prev_kf->node;
    e->vertices()[1] = kf->node;                     
    optimizer_.addEdge(e);
  }

  // Adding points
  addMapPoints(kf->points, kf->pose);
}

void Map::addLoopClosure(const int f1, const int f2, const Eigen::Isometry3d& rel_pose) {

  // Computing edge covariance
  Eigen::MatrixXd im = infcalc_.calcInfMatrix(keyframes_[f1]->points, keyframes_[f2]->points, rel_pose);

  g2o::EdgeSE3* e(new g2o::EdgeSE3());
  e->setMeasurement(rel_pose);
  //e->setMeasurement(t);
  e->setInformation(im);
  e->vertices()[0] = keyframes_[f1]->node;
  e->vertices()[1] = keyframes_[f2]->node;
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  rk->setDelta(1.0);
  e->setRobustKernel(rk);                    
  optimizer_.addEdge(e);

  // Adding the corresponding loop
  keyframes_[f2]->loops.push_back(f1);
}

void Map::optimize(const int iters) {
  
  // Optimize
  if (optimizer_.vertices().size() > 2) {
    optimizer_.initializeOptimization();
    optimizer_.optimize(iters);
  }  

  // Update the pose on every keyframe
  for (size_t i = 0; i < keyframes_.size(); i++) {
    keyframes_[i]->pose = keyframes_[i]->node->estimate();    
  }

  // Generate the new optimized map
  clearMapPoints();
  for (size_t i = 0; i < keyframes_.size(); i++) {
    addMapPoints(keyframes_[i]->points, keyframes_[i]->pose);
  }
}

void Map::clearMapPoints() {
  
  // Clear HashMap
  cells_.clear();

  // Remove dynamic memory
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    delete cells_vector_[i];
  }

  cells_vector_.clear();
}

void Map::addMapPoints(const PointCloud::Ptr& pc_in, const Eigen::Isometry3d& pose) {

  // Compute world coordinates
  PointCloud::Ptr points_map(new PointCloud);
  pcl::transformPointCloud(*pc_in, *points_map, pose.matrix());

  // Process each point
  for (size_t i = 0; i < points_map->points.size(); i++) {

    // Get the current point
		Point point = points_map->points[i];

		// Compute the corresponding cell
    int voxel_x = int(std::floor(point.x * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));
    int voxel_y = int(std::floor(point.y * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));
    int voxel_z = int(std::floor(point.z * inv_voxel_zsize_) * voxel_zsize_ + (voxel_zsize_half_));
    
    // Check if the voxel is already in the map
    HashKey key(voxel_x, voxel_y, voxel_z);
    Cell* cell;
    if (cells_.find(key) == cells_.end()) {
      // Cell is not in the map, so it is added
      Eigen::Vector3d p(voxel_x, voxel_y, voxel_z);
      cell = new Cell(p);
      cells_[key] = cell;
      cells_vector_.push_back(cell);
    } else {
      // Otherwise we get the corresponding cell
      cell = cells_[key];
    }

    cell->addPoint(point);
  }

  // Update every modified cell
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    if (cells_vector_[i]->modified()) {
      cells_vector_[i]->filter(vgrid_filter_);
    }
  }
}

PointCloud::Ptr Map::getMapPoints() {
  PointCloud::Ptr total_points(new PointCloud);
  // Get PCs from every cell and concatenate them
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    *total_points += *(cells_vector_[i]->getPoints());
  }

  return total_points;
}

PointCloud::Ptr Map::getLocalMapPoints(const Eigen::Isometry3d& pose, int cells_xy, int cells_z) {
  // Computing the voxel corresponding to the current position
  // Compute the corresponding cell
  int x = pose.translation().x();
  int voxel_x = int(std::floor(x * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));

  int y = pose.translation().y();
  int voxel_y = int(std::floor(y * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));

  int z = pose.translation().z();
  int voxel_z = int(std::floor(z * inv_voxel_zsize_) * voxel_zsize_ + (voxel_zsize_half_));

  // Final local map
  PointCloud::Ptr total_points(new PointCloud);

  // Getting points on neighbouring voxels of x and y
  int init_x = voxel_x - cells_xy * voxel_xysize_;
  int end_x =  voxel_x + cells_xy * voxel_xysize_;
  int init_y = voxel_y - cells_xy * voxel_xysize_;
  int end_y =  voxel_y + cells_xy * voxel_xysize_;
  // Get PCs from every cell on x axis
  for (int i = init_x; i <= end_x; i += voxel_xysize_) {
    for (int j = init_y; j <= end_y; j += voxel_xysize_) {
      // Check if the voxel is already in the map
      HashKey key(i, j, voxel_z);
      Cell* cell;
      if (cells_.find(key) != cells_.end()) {
        cell = cells_[key];
        *total_points += *(cell->getPoints());
      }
    }        
  }
    
  // Getting points on neighbouring voxels of z
  int init_z = voxel_z - cells_z * voxel_xysize_;
  int end_z  =  voxel_z + cells_z * voxel_xysize_;
  // Get PCs from every cell on x axis
  for (int i = init_z; i <= end_z; i += voxel_zsize_) {
    // Check if the voxel is already in the map
    HashKey key(voxel_x, voxel_y, i);
    Cell* cell;
    if (cells_.find(key) != cells_.end()) {
      cell = cells_[key];
      *total_points += *(cell->getPoints());
    }    
  }

  return total_points;
}

std::vector<Cell*>* Map::getCells() {
  return &cells_vector_;
}

std::vector<Keyframe*>* Map::getKeyframes() {
  return &keyframes_;
}

unsigned Map::existsKeyframe(int kf_id) {
  return ((unsigned)kf_id < keyframes_.size());
}

}  // namespace lihash_slam
