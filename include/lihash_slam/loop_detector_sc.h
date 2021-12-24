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

#ifndef INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_SC_H
#define INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_SC_H

#include <lihash_slam/loop_detector_base.hpp>
#include <Scancontext.h>

// PCL
#include <pcl/registration/gicp.h>

namespace lihash_slam {

// LC detector based on Scan Context
class LoopDetectorSC : public LoopDetector {
 public:
  LoopDetectorSC();
  virtual ~LoopDetectorSC();

  void readParams(const ros::NodeHandle& nh);
  void init();
  void addFrame(const int id, const Eigen::Isometry3d& pose, const PointCloud::Ptr& points);
  bool detect(Loop& loop);
 
 private:
  SCManager sc_man_;
  std::vector<LoopFrame> frames;

  // Params  
  double score_th_;
};

} // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_LOOP_DETECTOR_SC_H