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

#include <lihash_slam/loop_detector_pose.h>

namespace lihash_slam {

LoopDetectorPose::LoopDetectorPose() : 
  LoopDetector() {
}

LoopDetectorPose::~LoopDetectorPose() {
}

void LoopDetectorPose::readParams(const ros::NodeHandle& nh) {
  // TODO Read required params
}

void LoopDetectorPose::init() {
  // TODO Initialize class: ROS publishers, among others  
}

// Adding a new possible frame
void LoopDetectorPose::addFrame(const Eigen::Isometry3d& pose, const PointCloud::Ptr& points) {
  LoopFrame frame((int)frames.size(), pose, points);
  frames.push_back(frame);

  ROS_INFO("Frame added!");
}

void LoopDetectorPose::detect(const Eigen::Isometry3d& pose, const PointCloud::Ptr& points, std::vector<Loop>& loops) {
}

}  // namespace lihash_slam
