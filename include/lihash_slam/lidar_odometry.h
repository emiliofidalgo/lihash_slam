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

#ifndef INCLUDE_LIHASH_SLAM_LIDAR_ODOMETRY_H
#define INCLUDE_LIHASH_SLAM_LIDAR_ODOMETRY_H

#include <numeric>
#include <queue>

// Ceres
#include <ceres/ceres.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

// Liodom
#include <lihash_slam/defs.h>
#include <lihash_slam/factors.hpp>
#include <lihash_slam/KeyframeMessage.h>

namespace lihash_slam {

// Local Map manager
class LocalMapManager {
 public:
  explicit LocalMapManager(const size_t max_frames);
  virtual ~LocalMapManager();

  void addPointCloud(const PointCloud::Ptr& pc);
  size_t getLocalMap(PointCloud::Ptr& map);
  void setMaxFrames(const size_t max_nframes);

 private:
  PointCloud::Ptr total_points_;
  size_t nframes_;
  size_t max_nframes_;
  std::queue<size_t> sizes_;
};

// Lidar Odometry
class LidarOdometer {
 public:
  explicit LidarOdometer(const ros::NodeHandle& nh);
  virtual ~LidarOdometer();

  void process(const PointCloud::Ptr& pc_in, const std_msgs::Header& header);

 private:
  // ROS variables
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Publisher twist_pub_;
  ros::Publisher kf_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Variables
  bool init_;
  Eigen::Isometry3d prev_odom_;
  Eigen::Isometry3d odom_;
  Eigen::Isometry3d prev_kf_;
  Eigen::Isometry3d last_kf_;
  double prev_stamp_;
  double kf_stamp_;
  double param_q[4] = {0, 0, 0, 1};
  double param_t[3] = {0, 0, 0};
  Eigen::Isometry3d laser_to_base_;
  PointCloud::Ptr kf_points_;
  LocalMapManager lmap_;
  std::vector<Eigen::Isometry3d> kf_rel_poses_;

  // Params
  double min_range_;
  double max_range_;
  std::string fixed_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  bool publish_tf_;
  double kf_dist_;
  double kf_rot_;
  int kf_frames_;
  int lmap_frames_;
  int acc_frames_;

  void readParams();
  void addEdgeConstraints(const PointCloud::Ptr& edges,
                          const Eigen::Isometry3d& pose,
                          ceres::Problem* problem,
                          ceres::LossFunction* loss);
  bool getBaseToLidarTf(const std::string& frame_id);
  bool isNewKF(const Eigen::Isometry3d& pose);
  void publish(const std_msgs::Header& header, const Eigen::Isometry3d& pose);
};

}  // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_LIDAR_ODOMETRY_H
