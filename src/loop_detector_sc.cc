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

#include <lihash_slam/loop_detector_sc.h>

namespace lihash_slam {

LoopDetectorSC::LoopDetectorSC() : 
  LoopDetector() {
}

LoopDetectorSC::~LoopDetectorSC() {
}

void LoopDetectorSC::readParams(const ros::NodeHandle& nh) {
  // Best score thresh
  nh.param("score_th", score_th_, 4.0);
  ROS_INFO("Best score thresh: %.2f", score_th_);
}

void LoopDetectorSC::init() {
}

// Adding a new possible frame
void LoopDetectorSC::addFrame(const int id, const Eigen::Isometry3d& pose, const PointCloud::Ptr& points) {  
  
  int curr_id = id;
  LoopFrame frame(curr_id, Eigen::Isometry3d::Identity(), points);
  frames.push_back(frame);

  // Adding the frame to ScanContext
  // TODO: Downsample cloud?
  sc_man_.makeAndSaveScancontextAndKeys(*points);
}

// Detecting loops for the last added frame
bool LoopDetectorSC::detect(Loop& loop) {

  bool response = false;  

  // Detecting the loop
  std::pair<int, float> sc_loop = sc_man_.detectLoopClosureID();

  if (sc_loop.first > -1) {    
    // Get the last added frame to look for LCs
    LoopFrame* cur_frame = &(frames[frames.size() - 1]);

    // Validating candidates
    pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
    gicp.setTransformationEpsilon(0.1);
    gicp.setMaximumIterations(64);  
    gicp.setMaxCorrespondenceDistance(2.0);
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaximumOptimizerIterations(20);
    // Setting point cloud to be aligned
    gicp.setInputSource(cur_frame->points);
    // Setting point cloud to be aligned to
    gicp.setInputTarget(frames[sc_loop.first].points);

    // Computing the initial guess
    Eigen::Isometry3d init_guess = frames[sc_loop.first].pose.inverse() * cur_frame->pose;

    // Calculating required rigid transform to align the input cloud to the target cloud
    PointCloud::Ptr aligned(new PointCloud);
    gicp.align(*aligned, init_guess.matrix().cast<float>());
    double score = gicp.getFitnessScore();

    ROS_INFO("Score: %f", score);

    if (gicp.hasConverged() && score < score_th_) {
      Eigen::Isometry3d rel_pose;
      rel_pose = gicp.getFinalTransformation().cast<double>();
      // Filling data on loop struct
      loop.frame1 = frames[sc_loop.first].id;
      loop.frame2 = cur_frame->id;      
      loop.rel_pose = rel_pose;
      response = true;

      ROS_INFO("Loop found!!!!!!!!!!!!!!!!!!");   
      ROS_INFO("(%i, %i)", loop.frame1, loop.frame2);
    }
  }

  return response;
}

}  // namespace lihash_slam
