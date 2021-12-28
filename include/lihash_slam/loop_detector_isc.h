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

#ifndef INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_ISC_H
#define INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_ISC_H

#include <lihash_slam/loop_detector_base.hpp>
#include <iscGenerationClass.h>

// PCL
#include <pcl/registration/gicp.h>

// ROS
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace lihash_slam {

// LC detector based on Intensity Scan Context (see ISC-LOAM)
class LoopDetectorISC : public LoopDetector {
 public:
  LoopDetectorISC();
  virtual ~LoopDetectorISC();

  void readParams(const ros::NodeHandle& nh);
  void init();
  void addFrame(const int id, const Eigen::Isometry3d& pose, const PointCloud::Ptr& points);
  bool detect(Loop& loop);
 
 private:
  ISCGenerationClass gen_;
  ros::Publisher isc_pub_;
  std::vector<LoopFrame> frames;
  ros::NodeHandle nh_aux_;

  // Params
  int rings_;
  int sectors_;
  double max_dist_;
  double score_th_;
};

} // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_ISC_H