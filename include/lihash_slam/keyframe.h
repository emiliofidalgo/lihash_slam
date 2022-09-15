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

#ifndef INCLUDE_LIHASH_SLAM_KEYFRAME_H
#define INCLUDE_LIHASH_SLAM_KEYFRAME_H

#include <vector>

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>

// G2O
#include <g2o/types/slam3d/vertex_se3.h>

// LiHash-SLAM
#include <lihash_slam/defs.h>

namespace lihash_slam {

// Keyframe
class Keyframe {
 public:
  Keyframe();
  explicit Keyframe(const int id_, Eigen::Isometry3d& pose_, const PointCloud::Ptr& pc_);
  virtual ~Keyframe();

  void addFramePoses(const std::vector<geometry_msgs::Pose>& rel_poses);
 
  int id;
  Eigen::Isometry3d pose_est;   // Pose estimated
  Eigen::Isometry3d pose_odom;  // Odom pose
  PointCloud::Ptr points;       // Points in KF coords
  g2o::VertexSE3* node;
  ros::Time stamp_;
  std::vector<int> loops;
  std::vector<Eigen::Isometry3d> frame_poses;  
};

}  // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_KEYFRAME_H