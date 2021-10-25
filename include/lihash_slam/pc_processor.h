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

#ifndef INCLUDE_LIHASH_SLAM_PC_PROCESSOR_H
#define INCLUDE_LIHASH_SLAM_PC_PROCESSOR_H

// C++
#include <atomic>
#include <chrono>
#include <omp.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <lihash_slam/defs.h>

namespace lihash_slam {

// Smoothness
struct SmoothnessItem {

  inline SmoothnessItem() :
    point_index(-1),
    smoothness(-1.0) {};

  inline explicit SmoothnessItem(const int index, const double smooth) :
    point_index(index),
    smoothness(smooth) {};

  int point_index;
  double smoothness;

  bool operator<(const SmoothnessItem& s) const {
    return smoothness > s.smoothness;
  }
};

class PointCloudProcessor {
  public:  
    explicit PointCloudProcessor(const ros::NodeHandle& nh);
    virtual ~PointCloudProcessor();

    void process(const PointCloud::Ptr& pc_in, PointCloud::Ptr& pc_out);

  private:
    // ROS variables
    ros::NodeHandle nh_;

    // Variables
    bool picked_[400000];
    int ncores_;
    double min_range_;
    double max_range_;
    int lidar_type_;
    int scan_lines_;
    int scan_regions_;
    int edges_per_region_;
    size_t min_points_per_scan_;

    void readParams();
    bool isValidPoint(const double x, const double y, const double z, double* dist);
    void splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans);
    void extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges);
    void extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges);
};

}  // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_PC_PROCESSOR_H