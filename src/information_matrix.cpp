// SPDX-License-Identifier: BSD-2-Clause

#include <lihash_slam/information_matrix.h>

#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

namespace lihash_slam {

InformationMatrixCalculator::InformationMatrixCalculator() :
  var_gain_a_(20.0),
  min_stddev_x_(0.1),
  max_stddev_x_(5.0),
  min_stddev_q_(0.05),
  max_stddev_q_(0.2),
  fitness_score_thresh_(0.5) {
}

InformationMatrixCalculator::~InformationMatrixCalculator() {}

void InformationMatrixCalculator::readParams(const ros::NodeHandle& nh) {
  
  nh.param("var_gain_a", var_gain_a_, 20.0);
  nh.param("min_stddev_x", min_stddev_x_, 0.1);
  nh.param("max_stddev_x", max_stddev_x_, 5.0);
  nh.param("min_stddev_q", min_stddev_q_, 0.05);
  nh.param("max_stddev_q", max_stddev_q_, 0.2);
  nh.param("fitness_score_thresh", fitness_score_thresh_, 0.5);
}

Eigen::MatrixXd InformationMatrixCalculator::calcInfMatrix(const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2, const Eigen::Isometry3d& rel_pose) {

  double fitness_score = fitnessScore(cloud1, cloud2, rel_pose);

  double min_var_x = std::pow(min_stddev_x_, 2);
  double max_var_x = std::pow(max_stddev_x_, 2);
  double min_var_q = std::pow(min_stddev_q_, 2);
  double max_var_q = std::pow(max_stddev_q_, 2);

  float w_x = weight(var_gain_a_, fitness_score_thresh_, min_var_x, max_var_x, fitness_score);
  float w_q = weight(var_gain_a_, fitness_score_thresh_, min_var_q, max_var_q, fitness_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  inf.topLeftCorner(3, 3).array() /= w_x;
  inf.bottomRightCorner(3, 3).array() /= w_q;
  return inf;
}

double InformationMatrixCalculator::fitnessScore(const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2, const Eigen::Isometry3d& rel_pose, double max_range) {
  pcl::search::KdTree<Point>::Ptr tree_(new pcl::search::KdTree<Point>());
  tree_->setInputCloud(cloud1);

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloud input_transformed;
  pcl::transformPointCloud(*cloud2, input_transformed, rel_pose.cast<float>());

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);

  // For each point in the source dataset
  int nr = 0;
  for(size_t i = 0; i < input_transformed.points.size(); ++i) {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if(nn_dists[0] <= max_range) {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if(nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max());
}

double InformationMatrixCalculator::weight(double a, double max_x, double min_y, double max_y, double x) {
  double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
  return min_y + (max_y - min_y) * y;
}

}  // namespace lihash_slam
