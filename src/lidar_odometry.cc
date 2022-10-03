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

#include <lihash_slam/lidar_odometry.h>

namespace lihash_slam {

LocalMapManager::LocalMapManager(const size_t max_frames) :
  total_points_(new PointCloud),
  nframes_(0),
  max_nframes_(max_frames) {
}

LocalMapManager::~LocalMapManager() {
}

void LocalMapManager::addPointCloud(const PointCloud::Ptr& pc) {
  
  // Adding the current frame to the local map
  *total_points_ += *pc;
  nframes_++;
  sizes_.push(pc->size());

  // Removing frames at the beginning if required
  if (nframes_ > max_nframes_) {
    // Get the size of the first frame
    size_t pc_size = sizes_.front();
    sizes_.pop();

    // Remove the points corresponding to the first frame
    std::vector<int> ind(pc_size);
    std::iota(std::begin(ind), std::end(ind), 0);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices = ind;
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(total_points_);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*total_points_);

    // Reducing the number of frames
    nframes_--;
  }
}

size_t LocalMapManager::getLocalMap(PointCloud::Ptr& map) {
  map = total_points_;
  return nframes_;
}

void LocalMapManager::setMaxFrames(const size_t max_nframes) {
  max_nframes_ = max_nframes;
}

LidarOdometer::LidarOdometer(const ros::NodeHandle& nh) :
  nh_(nh),
  init_(false),
  prev_odom_(Eigen::Isometry3d::Identity()),
  odom_(Eigen::Isometry3d::Identity()),
  prev_kf_(Eigen::Isometry3d::Identity()),
  last_kf_(Eigen::Isometry3d::Identity()),
  prev_stamp_(0.0),
  kf_stamp_(0.0),
  kf_points_(new PointCloud),
  lmap_(15),
  lmap_frames_(15),
  acc_frames_(0) {

    readParams();

    // Publishers
    odom_pub_  = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    kf2b_pub_  = nh_.advertise<geometry_msgs::PoseStamped>("kf2b", 10);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist", 10);
    kf_pub_    = nh_.advertise<lihash_slam::KeyframeMessage>("kfs", 50);
}

LidarOdometer::~LidarOdometer() {
}

void LidarOdometer::readParams() {
  // Reading parameters
  // Minimum range in meters
  nh_.param("min_range", min_range_, 3.0);
  ROS_INFO("Minimum range: %.2f", min_range_);

  // Maximum range in meters
  nh_.param("max_range", max_range_, 75.0);
  ROS_INFO("Maximum range: %.2f", max_range_);

  // Fixed frame
  nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
  ROS_INFO("Fixed frame: %s", fixed_frame_.c_str());

  // Base frame
  nh_.param<std::string>("base_frame", base_frame_, "base_link");
  ROS_INFO("Base frame: %s", base_frame_.c_str());

  // Lidar frame
  nh_.param<std::string>("lidar_frame", lidar_frame_, "");
  if (lidar_frame_ == ""){
    ROS_INFO("Using lidar frame from header");
  }else{
    ROS_INFO("Lidar frame: %s", lidar_frame_.c_str());
  }

  // Publish TF
  nh_.param("publish_tf", publish_tf_, true);
  ROS_INFO("Publish TF: %s", publish_tf_ ? "Yes" : "No");

  // Maximum distance in meters to create a new KF
  nh_.param("kf_dist", kf_dist_, 15.0);
  ROS_INFO("New KF dist: %.2f", kf_dist_);

  // Maximum rotation in radians to create a new KF
  nh_.param("kf_rot", kf_rot_, 2.0);
  ROS_INFO("New KF rot: %.2f", kf_rot_);

  // Maximum number of frames to create a new KF
  nh_.param("kf_frames", kf_frames_, 10);
  ROS_INFO("New KF frames: %d", kf_frames_);

  // Number of frames in the local map
  nh_.param("lmap_frames", lmap_frames_, 15);
  ROS_INFO("Local map frames: %d", lmap_frames_);
  lmap_.setMaxFrames(lmap_frames_);
}

void LidarOdometer::process(const PointCloud::Ptr& pc_in, const std_msgs::Header& header) {
  
  if (!init_) {

    // Cache the static tf from base to laser (Tlb)
    if (lidar_frame_ == "") {
      lidar_frame_ = header.frame_id;
    }

    if (!getBaseToLidarTf(lidar_frame_)) {
      ROS_WARN("Skipping point_cloud");
      return;
    }

    // Initializing the local map
    lmap_.addPointCloud(pc_in);
    *kf_points_ += *pc_in;

    // Saving previous timestamp for computing the velocities
    prev_stamp_ = header.stamp.toSec();
    kf_stamp_   = header.stamp.toSec();

    publish(header, odom_);

    init_ = true;
  } else {        

    // Predict the current pose
    Eigen::Isometry3d pred_odom = odom_ * (prev_odom_.inverse() * odom_);
    prev_odom_ = odom_;
    odom_ = pred_odom;

    // Updating the initial guess
    Eigen::Quaterniond q_curr(odom_.rotation());
    param_q[0] = q_curr.x();
    param_q[1] = q_curr.y();
    param_q[2] = q_curr.z();
    param_q[3] = q_curr.w();

    Eigen::Vector3d t_curr = odom_.translation();
    param_t[0] = t_curr.x();
    param_t[1] = t_curr.y();
    param_t[2] = t_curr.z();

    // Optimize the current pose
    for (int optim_it = 0; optim_it < 2; optim_it++) {
      
      // Define the optimization problem
      ceres::LossFunction* loss_function = new ceres::HuberLoss(0.2);
      ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(param_q, 4, q_parameterization);
      problem.AddParameterBlock(param_t, 3);

      // Adding constraints
      addEdgeConstraints(pc_in, odom_, &problem, loss_function);

      // Solving the optimization problem
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );          
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);      
      // std::cout << summary.BriefReport() << "\n";

      odom_ = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q_new(param_q[3], param_q[0], param_q[1], param_q[2]);
      odom_.linear() = q_new.toRotationMatrix();
      odom_.translation() = Eigen::Vector3d(param_t[0], param_t[1], param_t[2]);
    }   

    // Compute the position of the detected edges in world coords
    PointCloud::Ptr edges_map(new PointCloud);
    pcl::transformPointCloud(*pc_in, *edges_map, odom_.matrix());

    // Add these points to the local map
    lmap_.addPointCloud(edges_map);

    // Checking if this should be a new KF
    if (!isNewKF(odom_)) {
      // Compute the position of the edges in keyframe coords
      Eigen::Isometry3d Tkl = prev_kf_.inverse() * odom_;
      PointCloud::Ptr edges_kf(new PointCloud);
      pcl::transformPointCloud(*pc_in, *edges_kf, Tkl.matrix());

      // Add these points to the keyframe points
      *kf_points_ += *edges_kf;

      // We increment the number of processed frames after a new keyframe
      acc_frames_++;

      // Accumulate the current pose wrt the KF
      kf_rel_poses_.push_back(Tkl);

      // Publishing poses
      publish(header, odom_);
      
    } else {
      //Estimating the relative pose of the current keyframe
      Eigen::Isometry3d rel_pose = last_kf_.inverse() * prev_kf_;
      
      // Publishing keyframe
      Eigen::Quaterniond q_current(rel_pose.rotation());
      Eigen::Vector3d t_current = rel_pose.translation();

      // Creating the base message
      lihash_slam::KeyframeMessage kf_msg;
      kf_msg.header.frame_id = "prev_keyframe";
      kf_msg.header.stamp.fromSec(kf_stamp_);

      // Filling the pose (relative) 
      kf_msg.pose.orientation.x = q_current.x();
      kf_msg.pose.orientation.y = q_current.y();
      kf_msg.pose.orientation.z = q_current.z();
      kf_msg.pose.orientation.w = q_current.w();
      kf_msg.pose.position.x = t_current.x();
      kf_msg.pose.position.y = t_current.y();
      kf_msg.pose.position.z = t_current.z();

      Eigen::Quaterniond q_current_op(prev_kf_.rotation());
      Eigen::Vector3d t_current_op = prev_kf_.translation();

      // Filling the pose (absolute) 
      kf_msg.odom_pose.orientation.x = q_current_op.x();
      kf_msg.odom_pose.orientation.y = q_current_op.y();
      kf_msg.odom_pose.orientation.z = q_current_op.z();
      kf_msg.odom_pose.orientation.w = q_current_op.w();
      kf_msg.odom_pose.position.x = t_current_op.x();
      kf_msg.odom_pose.position.y = t_current_op.y();
      kf_msg.odom_pose.position.z = t_current_op.z();   

      // Publishing the transform between odom and keyframe   
      // if (publish_tf_) {   
      //   Eigen::Quaterniond q_current_world(prev_kf_.rotation());
      //   Eigen::Vector3d t_current_world = prev_kf_.translation();
      //   tf::Transform transform;
      //   transform.setOrigin(tf::Vector3(t_current_world.x(), t_current_world.y(), t_current_world.z()));
      //   tf::Quaternion q(q_current_world.x(), q_current_world.y(), q_current_world.z(), q_current_world.w());
      //   transform.setRotation(q);
      //   ros::Time kf_time;
      //   kf_time.fromSec(kf_stamp_);
      //   tf_broadcaster_.sendTransform(tf::StampedTransform(transform, kf_time, fixed_frame_, "keyframe"));   
      // }

      // Filling points
      pcl::VoxelGrid<Point> voxel_filter;
      voxel_filter.setLeafSize(0.4, 0.4, 0.4);
      voxel_filter.setInputCloud(kf_points_);
      voxel_filter.filter(*kf_points_);
      pcl::toROSMsg(*kf_points_, kf_msg.points);

      // Filling frame relative poses
      for (size_t i = 0; i < kf_rel_poses_.size(); i++) {
        // Getting relative pose
        Eigen::Quaterniond q_current(kf_rel_poses_[i].rotation());
        Eigen::Vector3d t_current = kf_rel_poses_[i].translation();
        geometry_msgs::Pose rel_pose;
        rel_pose.orientation.x = q_current.x();
        rel_pose.orientation.y = q_current.y();
        rel_pose.orientation.z = q_current.z();
        rel_pose.orientation.w = q_current.w();
        rel_pose.position.x = t_current.x();
        rel_pose.position.y = t_current.y();
        rel_pose.position.z = t_current.z(); 

        // Adding this relative pose
        kf_msg.rel_poses.push_back(rel_pose);
      }

      // Publish the message
      kf_pub_.publish(kf_msg);  

      // Publishing poses
      publish(header, odom_);    

      // Create a new KF
      last_kf_ = prev_kf_;
      prev_kf_ = odom_;
      
      // Cleaning up the local map
      kf_points_->clear();
      *kf_points_ += *pc_in;

      // Resetting the number of received frames
      acc_frames_ = 0;

      // Updating the keyframe stamp
      kf_stamp_ = header.stamp.toSec();

      // Cleaning up the relative poses
      kf_rel_poses_.clear();
    }    
  }  
}

void LidarOdometer::addEdgeConstraints(const PointCloud::Ptr& edges,
                        const Eigen::Isometry3d& pose,
                        ceres::Problem* problem,
                        ceres::LossFunction* loss) {
  // Translate edges
  PointCloud::Ptr edges_map(new PointCloud);
  pcl::transformPointCloud(*edges, *edges_map, pose.matrix());

  // Getting the local map
  PointCloud::Ptr local_map(new PointCloud);
  lmap_.getLocalMap(local_map);

  // Trying to match against the local map
  int correct_matches = 0;
  pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>);
  tree->setInputCloud(local_map);
  for (size_t i = 0; i < edges_map->points.size(); i++) {
    std::vector<int> indices;
    std::vector<float> sq_dist;
    tree->nearestKSearch(edges_map->points[i], 5, indices, sq_dist);

    if (sq_dist[4] < 1.0) {
      std::vector<Eigen::Vector3d> near_corners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(local_map->points[indices[j]].x,
                            local_map->points[indices[j]].y,
                            local_map->points[indices[j]].z);
        center = center + tmp;
        near_corners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmp_zero_mean = near_corners[j] - center;
        cov_mat = cov_mat + tmp_zero_mean * tmp_zero_mean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);
      
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        // Set a correct match
        correct_matches++;
        Eigen::Vector3d curr_point(edges->points[i].x,
                                   edges->points[i].y,
                                   edges->points[i].z);

        Eigen::Vector3d pt_a(local_map->points[indices[0]].x,
                             local_map->points[indices[0]].y,
                             local_map->points[indices[0]].z);
      
        Eigen::Vector3d pt_b(local_map->points[indices[1]].x,
                             local_map->points[indices[1]].y,
                             local_map->points[indices[1]].z);

        ceres::CostFunction* cost_function = Point2LineFactor::create(curr_point, pt_a, pt_b, min_range_, max_range_);
        problem->AddResidualBlock(cost_function, loss, param_q, param_t);
      }
    }
  }

  ROS_DEBUG("Correct matchings: %i", correct_matches);
}

bool LidarOdometer::getBaseToLidarTf(const std::string& frame_id) {

  tf::TransformListener tf_listener;
  tf::StampedTransform laser_to_base_tf;

  try  {
    tf_listener.waitForTransform(frame_id, base_frame_, ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform(frame_id, base_frame_, ros::Time(0), laser_to_base_tf);
  } catch (tf::TransformException &ex) {
    ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    return false;
  }

  laser_to_base_.translation() = Eigen::Vector3d(laser_to_base_tf.getOrigin().x(),
                                                 laser_to_base_tf.getOrigin().y(),
                                                 laser_to_base_tf.getOrigin().z());
  Eigen::Quaterniond q_aux(laser_to_base_tf.getRotation().w(),
                           laser_to_base_tf.getRotation().x(),
                           laser_to_base_tf.getRotation().y(),
                           laser_to_base_tf.getRotation().z());
  laser_to_base_.linear() = q_aux.toRotationMatrix();

  return true;
}

bool LidarOdometer::isNewKF(const Eigen::Isometry3d& pose) {
  
  Eigen::Isometry3d dkf = prev_kf_.inverse() * pose;
  double dtrans = dkf.translation().norm();
  double drot = Eigen::AngleAxisd(dkf.linear()).angle();

  //ROS_INFO("KF Distance: %.2f", dtrans);
  //ROS_INFO("KF Rotation: %.2f", drot);
  //ROS_INFO("KF Frames: %d", acc_frames_);

  if (dtrans > kf_dist_ || drot > kf_rot_ || acc_frames_ > kf_frames_) {
    return true;
  } else {
    return false;
  }
}

void LidarOdometer::publish(const std_msgs::Header& header, const Eigen::Isometry3d& pose) {

  // Transform to base_link frame before publication  
  Eigen::Isometry3d odom_base_link = pose * laser_to_base_;

  Eigen::Quaterniond q_current(odom_base_link.rotation());
  Eigen::Vector3d t_current = odom_base_link.translation();

  // Publishing odometry
  nav_msgs::Odometry laser_odom_msg;
  laser_odom_msg.header.frame_id = fixed_frame_;
  laser_odom_msg.child_frame_id = base_frame_;
  laser_odom_msg.header.stamp = header.stamp;
  //Filling pose
  laser_odom_msg.pose.pose.orientation.x = q_current.x();
  laser_odom_msg.pose.pose.orientation.y = q_current.y();
  laser_odom_msg.pose.pose.orientation.z = q_current.z();
  laser_odom_msg.pose.pose.orientation.w = q_current.w();
  laser_odom_msg.pose.pose.position.x = t_current.x();
  laser_odom_msg.pose.pose.position.y = t_current.y();
  laser_odom_msg.pose.pose.position.z = t_current.z();

  //Filling twist
  if (init_) {
    double delta_time = header.stamp.toSec() - prev_stamp_;
    Eigen::Isometry3d delta_odom = ((prev_odom_ * laser_to_base_).inverse() * odom_base_link);
    Eigen::Vector3d t_delta = delta_odom.translation();
    laser_odom_msg.twist.twist.linear.x = t_delta.x() / delta_time;
    laser_odom_msg.twist.twist.linear.y = t_delta.y() / delta_time;
    laser_odom_msg.twist.twist.linear.z = t_delta.z() / delta_time;
    Eigen::Quaterniond q_delta(delta_odom.rotation());
    // We use tf because euler angles in Eigen present singularity problems
    tf::Quaternion quat(q_delta.x(), q_delta.y(), q_delta.z(), q_delta.w());
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    laser_odom_msg.twist.twist.angular.x = roll / delta_time;
    laser_odom_msg.twist.twist.angular.y = pitch / delta_time;
    laser_odom_msg.twist.twist.angular.z = yaw / delta_time;
  } else { // First frame
    laser_odom_msg.twist.twist.linear.x = 0.0;
    laser_odom_msg.twist.twist.linear.y = 0.0;
    laser_odom_msg.twist.twist.linear.z = 0.0;
    laser_odom_msg.twist.twist.angular.x = 0.0;
    laser_odom_msg.twist.twist.angular.y = 0.0;
    laser_odom_msg.twist.twist.angular.z = 0.0;    
  }
  prev_stamp_ = header.stamp.toSec();
  odom_pub_.publish(laser_odom_msg);

  // Publishing kTb
  Eigen::Isometry3d kf_2_b = prev_kf_.inverse() * pose * laser_to_base_;

  Eigen::Quaterniond q_current_kb(kf_2_b.rotation());
  Eigen::Vector3d t_current_kb = kf_2_b.translation();

  geometry_msgs::PoseStamped kf2b_pose;  
  kf2b_pose.header.frame_id = "keyframe";
  kf2b_pose.header.stamp = header.stamp;  
  kf2b_pose.pose.orientation.x = q_current_kb.x();
  kf2b_pose.pose.orientation.y = q_current_kb.y();
  kf2b_pose.pose.orientation.z = q_current_kb.z();
  kf2b_pose.pose.orientation.w = q_current_kb.w();
  kf2b_pose.pose.position.x = t_current_kb.x();
  kf2b_pose.pose.position.y = t_current_kb.y();
  kf2b_pose.pose.position.z = t_current_kb.z();

  kf2b_pub_.publish(kf2b_pose);

  // Publishing twist
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = base_frame_;
  twist_msg.header.stamp = header.stamp;
  twist_msg.twist = laser_odom_msg.twist.twist;
  twist_pub_.publish(twist_msg);

  //Publishing TF
  if (publish_tf_) {

    tf::Transform transform;

    // oTk
    Eigen::Quaterniond q_current_world(prev_kf_.rotation());
    Eigen::Vector3d t_current_world = prev_kf_.translation();    
    transform.setOrigin(tf::Vector3(t_current_world.x(), t_current_world.y(), t_current_world.z()));
    tf::Quaternion q(q_current_world.x(), q_current_world.y(), q_current_world.z(), q_current_world.w());
    transform.setRotation(q);    
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, header.stamp, fixed_frame_, "keyframe"));

    // kTb    
    transform.setOrigin(tf::Vector3(t_current_kb.x(), t_current_kb.y(), t_current_kb.z()));
    tf::Quaternion q_kb(q_current_kb.x(), q_current_kb.y(), q_current_kb.z(), q_current_kb.w());
    transform.setRotation(q_kb);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, header.stamp, "keyframe", base_frame_));
  }
}

}  // namespace lihash_slam
