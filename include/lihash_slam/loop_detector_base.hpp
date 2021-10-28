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

#ifndef INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_BASE_H
#define INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_BASE_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>

#include <lihash_slam/defs.h>

namespace lihash_slam {

// Class to represent a possible loop candidate in the LCD system
struct LoopFrame {
  explicit LoopFrame(const int id_, 
                     const Eigen::Isometry3d& pose_,
                     const PointCloud::Ptr& points_) :
    id(id_),
    pose(pose_),
    points(new PointCloud) {
      *points += *points_;
  }
 
  int id;
  Eigen::Isometry3d pose; // Pose in world coordinates
  PointCloud::Ptr points; // Associated points to this frame
};

// Class to represent an existent loop between two frames
struct Loop {
  explicit Loop() :
    frame1(-1),
    frame2(-1), 
    rel_pose(Eigen::Isometry3d::Identity()) {
  }  

  explicit Loop(const int f1, const int f2) : 
    frame1(f1),
    frame2(f2),
    rel_pose(Eigen::Isometry3d::Identity()) {
  }

  explicit Loop(const int f1, const int f2, const Eigen::Isometry3d& pose) : 
    frame1(f1),
    frame2(f2),
    rel_pose(pose) {
  }
  
  int frame1;
  int frame2;
  Eigen::Isometry3d rel_pose; // Estimated relative pose to move points from f1 to f2 (Tf2f1)
};

// Base class to create a LC detector
class LoopDetector {
 public:
  LoopDetector() {};
  virtual ~LoopDetector() {};

  virtual void readParams(const ros::NodeHandle& nh) = 0;
  virtual void init() = 0;
  virtual void addFrame(const int id, const Eigen::Isometry3d& pose, const PointCloud::Ptr& points) = 0;
  virtual bool detect(Loop& loop) = 0;
};

} // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_BASE_H