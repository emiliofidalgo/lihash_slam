/*
* This file is part of lihash_slam.
*
* Copyright (C) 2022 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
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

#ifndef INCLUDE_LIHASH_SLAM_KEYFRAME_MAKER_H
#define INCLUDE_LIHASH_SLAM_KEYFRAME_MAKER_H

#include <numeric>
#include <queue>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// LIHASH
#include <lihash_slam/defs.h>
#include <lihash_slam/KeyframeMessage.h>

namespace lihash_slam {

// KeyframeMaker
class KeyframeMaker {
 public:
  explicit KeyframeMaker(const ros::NodeHandle& nh);
  virtual ~KeyframeMaker();

  void process(const std_msgs::Header& header, const Eigen::Isometry3d& pose, const PointCloud::Ptr& pc_in);

 private:
  // ROS variables
  ros::NodeHandle nh_;
  ros::Publisher o2k_pub_;  
  ros::Publisher k2b_pub_;
  ros::Publisher kf_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Variables
  bool init_;
  Eigen::Isometry3d prev_kf_;
  Eigen::Isometry3d curr_kf_;
  double prev_stamp_;
  double kf_stamp_;
  int acc_frames_;

  Eigen::Isometry3d laser_to_base_;
  Eigen::Isometry3d base_to_laser_;
  PointCloud::Ptr kf_points_;
  std::vector<Eigen::Isometry3d> kf_rel_poses_;
  std::vector<uint64_t> kf_rel_stamps_;

  // Params  
  std::string fixed_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  bool publish_tf_;
  double kf_dist_;
  double kf_rot_;
  int kf_frames_;  

  void readParams();
  bool getBaseToLidarTf(const std::string& frame_id);
  bool isNewKF(const Eigen::Isometry3d& pose);
  void publish(const std_msgs::Header& header, const Eigen::Isometry3d& pose);
};

}  // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_KEYFRAME_MAKER_H
