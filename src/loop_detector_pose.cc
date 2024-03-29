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
  
  // Distance LCD thresh
  nh.param("dist_th", dist_th_, 25.0);
  ROS_INFO("Distance thresh: %.2f", dist_th_);

  // Accumulate distance LCD thresh
  nh.param("accum_dist_th", accum_dist_th_, 25.0);
  ROS_INFO("Accum distance thresh: %.2f", accum_dist_th_);

  // Best score thresh
  nh.param("score_th", score_th_, 2.5);
  ROS_INFO("Best score thresh: %.2f", score_th_);
}

void LoopDetectorPose::init() {
}

// Adding a new possible frame
void LoopDetectorPose::addFrame(const int id, const Eigen::Isometry3d& pose, const PointCloud::Ptr& points) {
  
  int curr_id = id;

  // Computing the absolute pose of the frame
  Eigen::Isometry3d abs_pose;
  if (frames.size() > 0) {
    abs_pose = frames[frames.size() - 1].pose * pose;
  } else {
    abs_pose = Eigen::Isometry3d::Identity();
  }

  LoopFrame frame(curr_id, abs_pose, points);
  frames.push_back(frame);

  if (curr_id == 0) { // If this is the first frame
    acc_dists.push_back(0.0);
  } else {
    // Compute the distance between this frame and the previous one
    //Eigen::Isometry3d T12 = frames[curr_id - 1].pose.inverse() * pose;    
    double d_trans = pose.translation().norm();

    // Track the accumulated distance on each frame
    acc_dists.push_back(acc_dists[curr_id - 1] + d_trans);
  }
}

// Detecting loops for the last added frame
bool LoopDetectorPose::detect(Loop& loop) {

  bool response = false;
  
  // Get the last added frame to look for LCs
  LoopFrame* cur_frame = &(frames[frames.size() - 1]);
  double cur_acc_dist = acc_dists[frames.size() - 1];

  // Finding loop candidates
  std::vector<LoopFrame*> candidates;
  for (unsigned i = 0; i < frames.size() - 1; i++) {
    // Check the accumulated distance between frames
    if (cur_acc_dist - acc_dists[i] < accum_dist_th_) {
      continue;
    }

    // Check the distance between frames
    double dist = (frames[i].pose.inverse() * cur_frame->pose).translation().norm();
    if (dist > dist_th_) {
      continue;
    }

    // Adding this frame as a possible loop
    candidates.push_back(&(frames[i]));
  }

  ROS_INFO("--- Loop Detection ---");
  ROS_INFO("Number of candidates: %d", (int)candidates.size());  

  std::vector<double> scores(candidates.size(), std::numeric_limits<double>::max());
  std::vector<Eigen::Isometry3d> rel_poses(candidates.size(), Eigen::Isometry3d::Identity());

  #pragma omp parallel for
  for (unsigned i = 0; i < candidates.size(); i++) {

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
    gicp.setInputTarget(candidates[i]->points);

    // Computing the initial guess
    Eigen::Isometry3d init_guess = candidates[i]->pose.inverse() * cur_frame->pose;

    // Calculating required rigid transform to align the input cloud to the target cloud
    PointCloud::Ptr aligned(new PointCloud);
    gicp.align(*aligned, init_guess.matrix().cast<float>());

    double score = gicp.getFitnessScore();
    ROS_INFO("Cand %i: %f", int(i), score);
    if (gicp.hasConverged()) {
      scores[i] = score;
      rel_poses[i] = gicp.getFinalTransformation().cast<double>();
    }
  }

  double best_score = std::numeric_limits<double>::max();
  LoopFrame* best_candidate = 0;
  Eigen::Isometry3d rel_pose;
  for (unsigned i = 0; i < candidates.size(); i++) {
    if (scores[i] < best_score) {
      best_score = scores[i];
      best_candidate = candidates[i];
      rel_pose = rel_poses[i];
    }
  }

  // Checking the final candidate
  if (best_score > score_th_) {
    ROS_INFO("Loop not found");
  } else {    
    // Filling data on loop struct
    loop.frame1 = best_candidate->id;
    loop.frame2 = cur_frame->id;    
    loop.rel_pose = rel_pose;
    response = true;

    ROS_INFO("Loop found!!!!!!!!!!!!!!!!!!");   
    ROS_INFO("(%i, %i)", loop.frame1, loop.frame2);
  }

  return response;
}

}  // namespace lihash_slam
