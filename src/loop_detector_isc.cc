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

#include <lihash_slam/loop_detector_isc.h>

namespace lihash_slam {

LoopDetectorISC::LoopDetectorISC() : 
  LoopDetector() {
}

LoopDetectorISC::~LoopDetectorISC() {
}

void LoopDetectorISC::readParams(const ros::NodeHandle& nh) {
  
  // Rings
  nh.param("rings", rings_, 60);
  ROS_INFO("Rings: %d", rings_);

  // Accumulate distance LCD thresh
  nh.param("sectors", sectors_, 60);
  ROS_INFO("Sectors: %d", sectors_);

  // Best score thresh
  nh.param("max_dist", max_dist_, 40.0);
  ROS_INFO("Maximum Distance: %.2f", max_dist_);

  // Best score thresh
  nh.param("score_th", score_th_, 4.0);
  ROS_INFO("Best score thresh: %.2f", score_th_);

  nh_aux_ = nh;
  isc_pub_ = nh_aux_.advertise<sensor_msgs::Image>("/isc_img", 100);
}

void LoopDetectorISC::init() {

  gen_.init_param(rings_, sectors_, max_dist_);
}

// Adding a new possible frame
void LoopDetectorISC::addFrame(const int id, const Eigen::Isometry3d& pose, const PointCloud::Ptr& points) {

  // Computing the absolute pose of the frame
  Eigen::Isometry3d abs_pose;
  if (frames.size() > 0) {
    abs_pose = frames[frames.size() - 1].pose * pose;
  } else {
    abs_pose = Eigen::Isometry3d::Identity();
  }
  
  int curr_id = id;
  LoopFrame frame(curr_id, abs_pose, points);
  frames.push_back(frame);
}

// Detecting loops for the last added frame
bool LoopDetectorISC::detect(Loop& loop) {

  bool response = false;
  
  // Get the last added frame to look for LCs
  LoopFrame* cur_frame = &(frames[frames.size() - 1]);

  gen_.loopDetection(cur_frame->points, cur_frame->pose);

  if (isc_pub_.getNumSubscribers() > 0) {
    cv_bridge::CvImage out_msg;
    out_msg.header.frame_id  = "isc"; 
    out_msg.header.stamp  = ros::Time::now();
    out_msg.encoding = sensor_msgs::image_encodings::RGB8; 
    out_msg.image = gen_.getLastISCRGB(); 
    isc_pub_.publish(out_msg.toImageMsg());
  }

  if (gen_.matched_frame_id.size() > 0) {    
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
    gicp.setInputTarget(frames[gen_.matched_frame_id[0]].points);

    // Computing the initial guess
    Eigen::Isometry3d init_guess = frames[gen_.matched_frame_id[0]].pose.inverse() * cur_frame->pose;

    // Calculating required rigid transform to align the input cloud to the target cloud
    PointCloud::Ptr aligned(new PointCloud);
    gicp.align(*aligned, init_guess.matrix().cast<float>());
    double score = gicp.getFitnessScore();

    ROS_INFO("Score: %f", score);

    if (gicp.hasConverged() && score < score_th_) {
      Eigen::Isometry3d rel_pose;
      rel_pose = gicp.getFinalTransformation().cast<double>();
      // Filling data on loop struct
      loop.frame1 = gen_.matched_frame_id[0];
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
