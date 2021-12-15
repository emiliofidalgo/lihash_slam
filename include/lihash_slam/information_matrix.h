// SPDX-License-Identifier: BSD-2-Clause

// Based on HDL Graph SLAM: https://github.com/koide3/hdl_graph_slam

#ifndef INCLUDE_LIHASH_SLAM_INFORMATION_MATRIX_H
#define INCLUDE_LIHASH_SLAM_INFORMATION_MATRIX_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <lihash_slam/defs.h>

#include <ros/ros.h>

namespace lihash_slam {

class InformationMatrixCalculator {
public:

  InformationMatrixCalculator();
  ~InformationMatrixCalculator();

  void readParams(const ros::NodeHandle& nh);
  Eigen::MatrixXd calcInfMatrix(const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2, const Eigen::Isometry3d& rel_pose);

private:
  double weight(double a, double max_x, double min_y, double max_y, double x);
  double fitnessScore(const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2, const Eigen::Isometry3d& rel_pose, double max_range = std::numeric_limits<double>::max());  
  
  double var_gain_a_;
  double min_stddev_x_;
  double max_stddev_x_;
  double min_stddev_q_;
  double max_stddev_q_;
  double fitness_score_thresh_;
};

}  // namespace lihash_slam

#endif  // INFORMATION_MATRIX_CALCULATOR_HPP
