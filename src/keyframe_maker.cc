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

#include <lihash_slam/keyframe_maker.h>

namespace lihash_slam {

KeyframeMaker::KeyframeMaker(const ros::NodeHandle& nh) :
  nh_(nh),
  init_(false),
  prev_kf_(Eigen::Isometry3d::Identity()),
  curr_kf_(Eigen::Isometry3d::Identity()),
  prev_stamp_(0.0),
  kf_stamp_(0.0),
  acc_frames_(0),
  kf_points_(new PointCloud),
  curr_kf_id_(0) {

    readParams();

    // Publishers
    o2k_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("o2k", 10);
    k2b_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("k2b", 10);
    kf_pub_    = nh_.advertise<lihash_slam::KeyframeMessage>("kfs", 50);
}

KeyframeMaker::~KeyframeMaker() {
}

void KeyframeMaker::readParams() {
  // Reading parameters

  // Fixed frame
  nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
  ROS_INFO("Fixed frame: %s", fixed_frame_.c_str());

  // Base frame
  nh_.param<std::string>("base_frame", base_frame_, "base_link");
  ROS_INFO("Base frame: %s", base_frame_.c_str());

  // Lidar frame
  nh_.param<std::string>("laser_frame", lidar_frame_, "");
  if (lidar_frame_ == "") {
    ROS_INFO("Using lidar frame from header");
  } else {
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
}

void KeyframeMaker::process(const std_msgs::Header& header, const Eigen::Isometry3d& pose, const PointCloud::Ptr& pc_in) {
  
  if (!init_) {

    // Cache the static tf from base to laser (Tlb)
    if (lidar_frame_ == "") {
      lidar_frame_ = header.frame_id;
    }

    if (!getBaseToLidarTf(lidar_frame_)) {
      ROS_WARN("Skipping point_cloud");
      return;
    }    

    // Saving the current KF as the initial one
    curr_kf_ = pose;

    // Saving previous timestamp
    prev_stamp_ = header.stamp.toSec();
    kf_stamp_   = header.stamp.toSec();

    // Tranforming the points to the base frame
    PointCloud::Ptr edges_base(new PointCloud);
    pcl::transformPointCloud(*pc_in, *edges_base, base_to_laser_.matrix());

    // Initializing the points in base coordinates
    *kf_points_ += *edges_base;
    kf_rel_poses_.clear();
    kf_rel_stamps_.clear();

    publish(header, pose);

    init_ = true;

  } else {

    // Checking if this should be a new KF
    if (!isNewKF(pose)) {
      // Compute the position of the edges in keyframe coords
      Eigen::Isometry3d Tkl = curr_kf_.inverse() * pose * base_to_laser_;
      PointCloud::Ptr edges_kf(new PointCloud);
      pcl::transformPointCloud(*pc_in, *edges_kf, Tkl.matrix());

      // Add these points to the keyframe points
      *kf_points_ += *edges_kf;

      // We increment the number of processed frames after a new keyframe
      acc_frames_++;

      // Accumulate the current pose wrt the KF
      Eigen::Isometry3d Tkb = curr_kf_.inverse() * pose;
      kf_rel_poses_.push_back(Tkb);
      kf_rel_stamps_.push_back(header.stamp.toNSec());

      // Publishing poses
      publish(header, pose);
      
    } else {
      //Estimating the relative pose of the current keyframe
      Eigen::Isometry3d rel_pose = prev_kf_.inverse() * curr_kf_;
      
      // Publishing keyframe
      Eigen::Quaterniond q_current(rel_pose.rotation());
      Eigen::Vector3d t_current = rel_pose.translation();

      // Creating the base message
      lihash_slam::KeyframeMessage kf_msg;
      // Set frame id according to the curr_kf_id_ using 5 digits
      std::stringstream ss;
      ss << "kf_" << std::setfill('0') << std::setw(7) << curr_kf_id_;
      kf_msg.header.frame_id = ss.str();
      // kf_msg.header.frame_id = "prev_keyframe";
      kf_msg.header.stamp.fromSec(kf_stamp_);

      // Filling the pose (relative) 
      kf_msg.rel_pose.orientation.x = q_current.x();
      kf_msg.rel_pose.orientation.y = q_current.y();
      kf_msg.rel_pose.orientation.z = q_current.z();
      kf_msg.rel_pose.orientation.w = q_current.w();
      kf_msg.rel_pose.position.x = t_current.x();
      kf_msg.rel_pose.position.y = t_current.y();
      kf_msg.rel_pose.position.z = t_current.z();

      Eigen::Quaterniond q_current_op(curr_kf_.rotation());
      Eigen::Vector3d t_current_op = curr_kf_.translation();

      // Filling the pose (absolute) 
      kf_msg.odom_pose.orientation.x = q_current_op.x();
      kf_msg.odom_pose.orientation.y = q_current_op.y();
      kf_msg.odom_pose.orientation.z = q_current_op.z();
      kf_msg.odom_pose.orientation.w = q_current_op.w();
      kf_msg.odom_pose.position.x = t_current_op.x();
      kf_msg.odom_pose.position.y = t_current_op.y();
      kf_msg.odom_pose.position.z = t_current_op.z();

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
        kf_msg.rel_stamps.push_back(kf_rel_stamps_[i]);
      }

      // Publish the message
      kf_pub_.publish(kf_msg);      

      // Create a new KF
      prev_kf_ = curr_kf_;
      curr_kf_ = pose;   

      // Update the current keyframe id
      curr_kf_id_++;   

      // Publishing poses
      publish(header, pose);      
      
      // Cleaning up the points
      kf_points_->clear();
      // Tranforming the points to the base frame
      PointCloud::Ptr edges_base(new PointCloud);
      pcl::transformPointCloud(*pc_in, *edges_base, base_to_laser_.matrix());

      *kf_points_ += *edges_base;

      // Resetting the number of received frames
      acc_frames_ = 1;

      // Updating the keyframe stamp
      kf_stamp_ = header.stamp.toSec();

      // Cleaning up the relative poses
      kf_rel_poses_.clear();
      kf_rel_stamps_.clear();
    }
  }  
}

bool KeyframeMaker::getBaseToLidarTf(const std::string& frame_id) {

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
  base_to_laser_ = laser_to_base_.inverse();

  return true;
}

bool KeyframeMaker::isNewKF(const Eigen::Isometry3d& pose) {
  
  Eigen::Isometry3d dkf = curr_kf_.inverse() * pose;
  double dtrans = dkf.translation().norm();
  double drot = Eigen::AngleAxisd(dkf.linear()).angle();

  //ROS_INFO("KF Distance: %.2f", dtrans);
  //ROS_INFO("KF Rotation: %.2f", drot);
  //ROS_INFO("KF Frames: %d", acc_frames_);

  if (dtrans > kf_dist_ || drot > kf_rot_ || ((acc_frames_ > kf_frames_) && (kf_frames_ > 0))) {
    return true;
  } else {
    return false;
  }
}

void KeyframeMaker::publish(const std_msgs::Header& header, const Eigen::Isometry3d& pose) {

  // Publishing oTk-1
  Eigen::Quaterniond q_current_ok(prev_kf_.rotation());
  Eigen::Vector3d t_current_ok = prev_kf_.translation();

  geometry_msgs::PoseStamped o2k_pose;
  o2k_pose.header.frame_id = fixed_frame_;
  o2k_pose.header.stamp = header.stamp;
  o2k_pose.pose.orientation.x = q_current_ok.x();
  o2k_pose.pose.orientation.y = q_current_ok.y();
  o2k_pose.pose.orientation.z = q_current_ok.z();
  o2k_pose.pose.orientation.w = q_current_ok.w();
  o2k_pose.pose.position.x = t_current_ok.x();
  o2k_pose.pose.position.y = t_current_ok.y();
  o2k_pose.pose.position.z = t_current_ok.z();
  o2k_pub_.publish(o2k_pose);

  // Publishing k-1Tb
  Eigen::Isometry3d kf_2_b = prev_kf_.inverse() * pose;
  Eigen::Quaterniond q_current_kb(kf_2_b.rotation());
  Eigen::Vector3d t_current_kb = kf_2_b.translation();

  geometry_msgs::PoseStamped kf2b_pose; 
  // Set frame id according to the curr_kf_id_ using 5 digits
  std::stringstream ss;
  if (curr_kf_id_ > 0) {
    ss << "kf_" << std::setfill('0') << std::setw(7) << (curr_kf_id_ - 1);
  } else {
    ss << "kf_" << std::setfill('0') << std::setw(7) << 0;
  }
  kf2b_pose.header.frame_id = ss.str();
  // kf2b_pose.header.frame_id = "prev_keyframe";
  kf2b_pose.header.stamp = header.stamp;  
  kf2b_pose.pose.orientation.x = q_current_kb.x();
  kf2b_pose.pose.orientation.y = q_current_kb.y();
  kf2b_pose.pose.orientation.z = q_current_kb.z();
  kf2b_pose.pose.orientation.w = q_current_kb.w();
  kf2b_pose.pose.position.x = t_current_kb.x();
  kf2b_pose.pose.position.y = t_current_kb.y();
  kf2b_pose.pose.position.z = t_current_kb.z();
  k2b_pub_.publish(kf2b_pose);

  //Publishing TF
  if (publish_tf_) {

    tf::Transform transform;

    // oTk
    transform.setOrigin(tf::Vector3(t_current_ok.x(), t_current_ok.y(), t_current_ok.z()));
    tf::Quaternion q(q_current_ok.x(), q_current_ok.y(), q_current_ok.z(), q_current_ok.w());
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
