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

#include <deque>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// LiHash SLAM
#include <lihash_slam/defs.h>
#include <lihash_slam/KeyframeMessage.h>
#include <lihash_slam/map.h>
#include <lihash_slam/LoopClosure.h>

// Message queues
std::deque<lihash_slam::Keyframe*> queue_kfs;
std::deque<lihash_slam::LoopClosure> queue_lcs;

// Map structure
lihash_slam::Map* map;

// Common params
std::string map_frame;
std::string odom_frame;
bool publish_tf;
double tf_period;
double cell_xy_size;
double cell_z_size;
bool save_results;
// Visualization params
double viz_kf_size;

// ROS
ros::Publisher map_points_pub;
ros::Publisher map_kfs_pub;
ros::Publisher map_cells_pub;
ros::Publisher map_traj_pub;
ros::Publisher map_traj_path_pub;

// Transforms
Eigen::Isometry3d mTo;    // Correction computed by SLAM
tf::Transform mTo_tf;

void publishKeyframes() {

  ros::Time time = ros::Time::now();  
  
  // Publishing keyframes
  if (map_kfs_pub.getNumSubscribers() > 0) {
    // Keyframes
    visualization_msgs::Marker marker_kfs;
    marker_kfs.header.frame_id = map_frame;
    marker_kfs.header.stamp = time;
    marker_kfs.ns = "kfs";
    marker_kfs.id = 0;
    marker_kfs.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_kfs.action = visualization_msgs::Marker::ADD;
    marker_kfs.scale.x = viz_kf_size;
    marker_kfs.scale.y = viz_kf_size;
    marker_kfs.scale.z = viz_kf_size;
    marker_kfs.color.a = 0.5;
    marker_kfs.color.b = 1.0;
    marker_kfs.pose.orientation.w = 1.0;

    // Links between them
    visualization_msgs::Marker marker_links;
    marker_links.header.frame_id = map_frame;
    marker_links.header.stamp = time;
    marker_links.ns = "kfs";
    marker_links.id = 1;
    marker_links.type = visualization_msgs::Marker::LINE_STRIP;
    marker_links.action = visualization_msgs::Marker::ADD;
    marker_links.scale.x = std::max(viz_kf_size - 1.0, 0.05);
    marker_links.color.a = 0.5;    
    marker_links.color.b = 1.0;
    marker_links.pose.orientation.w = 1.0;

    // Loops
    visualization_msgs::Marker marker_loops;
    marker_loops.header.frame_id = map_frame;
    marker_loops.header.stamp = time;
    marker_loops.ns = "kfs";
    marker_loops.id = 2;
    marker_loops.type = visualization_msgs::Marker::LINE_LIST;
    marker_loops.action = visualization_msgs::Marker::ADD;
    marker_loops.scale.x = std::max(viz_kf_size - 1.0, 0.05);
    marker_loops.color.a = 0.5;    
    marker_loops.color.r = 1.0;
    marker_loops.pose.orientation.w = 1.0;

    std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();
    marker_kfs.points.resize(kfs->size());
    marker_links.points.resize(kfs->size());
    for (size_t i = 0; i < kfs->size(); i++) {
      Eigen::Vector3d pose = kfs->at(i)->pose_est.translation();
      marker_kfs.points[i].x = pose.x();
      marker_kfs.points[i].y = pose.y();
      marker_kfs.points[i].z = pose.z();      
      marker_kfs.colors.push_back(marker_kfs.color);

      marker_links.points[i].x = pose.x();
      marker_links.points[i].y = pose.y();
      marker_links.points[i].z = pose.z();
      marker_links.colors.push_back(marker_links.color);

      // Checking loops with this keyframe
      for (size_t j = 0; j < kfs->at(i)->loops.size(); j++) {
        geometry_msgs::Point pose1;        
        pose1.x = pose.x();
        pose1.y = pose.y();
        pose1.z = pose.z();

        // LC
        int cand_ind = kfs->at(i)->loops[j];
        geometry_msgs::Point pose2;
        pose2.x = kfs->at(cand_ind)->pose_est.translation().x();
        pose2.y = kfs->at(cand_ind)->pose_est.translation().y();
        pose2.z = kfs->at(cand_ind)->pose_est.translation().z();        

        marker_loops.points.push_back(pose1);
        marker_loops.points.push_back(pose2);
        marker_loops.colors.push_back(marker_loops.color);
        marker_loops.colors.push_back(marker_loops.color);
      }
    }

    // Marker array
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker_kfs);
    //markers.markers.push_back(marker_links);
    if (marker_loops.points.size() > 0) {
      markers.markers.push_back(marker_loops);
    }

    map_kfs_pub.publish(markers);
  }
}

void publishMap(const ros::TimerEvent& event) {

  ros::Time time = ros::Time::now();

  publishKeyframes();
  
  // Publishing the current map
  if (map_points_pub.getNumSubscribers() > 0) {
    lihash_slam::PointCloud::Ptr m = map->getMapPoints();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*m, cloud_msg);
    cloud_msg.header.frame_id = map_frame;
    cloud_msg.header.stamp = time;
    map_points_pub.publish(cloud_msg);
  }

  // Publishing cells
  if (map_cells_pub.getNumSubscribers() > 0) {
    // Creating the template msg
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = time;
    marker.ns = "cells";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = cell_xy_size;
    marker.scale.y = cell_xy_size;
    marker.scale.z = cell_z_size;
    marker.color.a = 0.05;
    marker.color.g = 1.0;
    marker.pose.orientation.w = 1.0;

    std::vector<lihash_slam::Cell*>* cells = map->getCells();
    marker.points.resize(cells->size());
    for (size_t i = 0; i < cells->size(); i++) {
      Eigen::Vector3d pose = cells->at(i)->getPose();
      marker.points[i].x = pose.x();
      marker.points[i].y = pose.y();
      marker.points[i].z = pose.z();
      marker.colors.push_back(marker.color);
    }

    // Publish the marker of cells
    map_cells_pub.publish(marker);
  }  

  // Publishing the corrected trajectory
  if (map_traj_pub.getNumSubscribers()) {
    visualization_msgs::Marker marker_traj;
    marker_traj.header.frame_id = map_frame;
    marker_traj.header.stamp = time;
    marker_traj.ns = "traj";
    marker_traj.id = 0;
    marker_traj.type = visualization_msgs::Marker::LINE_STRIP;
    marker_traj.action = visualization_msgs::Marker::ADD;
    marker_traj.scale.x = std::max(viz_kf_size - 1.0, 0.05);
    marker_traj.color.a = 0.5;    
    marker_traj.color.r = 1.0;
    marker_traj.color.g = 1.0;
    marker_traj.pose.orientation.w = 1.0;

    std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();    
    for (size_t i = 0; i < kfs->size(); i++) {
      // Getting the KF pose
      Eigen::Isometry3d pose = kfs->at(i)->pose_est;

      // Adding the pose to the list
      geometry_msgs::Point kf_p;
      kf_p.x = pose.translation().x();
      kf_p.y = pose.translation().y();
      kf_p.z = pose.translation().z();
      marker_traj.points.push_back(kf_p);
      marker_traj.colors.push_back(marker_traj.color);

      // Iterating through each frame
      for (size_t j = 0; j < kfs->at(i)->frame_poses.size(); j++) {
        // Get the frame pose
        Eigen::Isometry3d frame_pose = kfs->at(i)->frame_poses[j];

        // Transform the frame pose
        Eigen::Isometry3d corr_pose = pose * frame_pose;

        // Adding the pose to the list
        geometry_msgs::Point kf_p;
        kf_p.x = corr_pose.translation().x();
        kf_p.y = corr_pose.translation().y();
        kf_p.z = corr_pose.translation().z();
        marker_traj.points.push_back(kf_p);
        marker_traj.colors.push_back(marker_traj.color);
      }
    }

    map_traj_pub.publish(marker_traj);
  }

  if (map_traj_path_pub.getNumSubscribers()) {

    // Fill a nav_msgs path message in ROS
    nav_msgs::Path path;
    path.header.frame_id = map_frame;
    path.header.stamp = time;

    std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();
    for (size_t i = 0; i < kfs->size(); i++) {
      // Getting the KF pose
      Eigen::Isometry3d pose = kfs->at(i)->pose_est;

      // Adding the pose to the list
      geometry_msgs::Point kf_p;
      kf_p.x = pose.translation().x();
      kf_p.y = pose.translation().y();
      kf_p.z = pose.translation().z();
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.pose.position = kf_p;
      path.poses.push_back(pose_stamped);

      // Iterating through each frame
      for (size_t j = 0; j < kfs->at(i)->frame_poses.size(); j++) {
        // Get the frame pose
        Eigen::Isometry3d frame_pose = kfs->at(i)->frame_poses[j];

        // Transform the frame pose
        Eigen::Isometry3d corr_pose = pose * frame_pose;

        // Adding the pose to the list
        geometry_msgs::Point kf_p;
        kf_p.x = corr_pose.translation().x();
        kf_p.y = corr_pose.translation().y();
        kf_p.z = corr_pose.translation().z();
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position = kf_p;
        path.poses.push_back(pose_stamped);
      }
    }

    map_traj_path_pub.publish(path);
  }
}

void writeResults(const std::string& results_file) {

  // Getting corrected poses
  std::vector<Eigen::Matrix4d> poses;
  std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();
  for (size_t i = 0; i < kfs->size(); i++) {
    // Getting the KF pose
    Eigen::Isometry3d pose_iso = kfs->at(i)->pose_est;
    Eigen::Matrix4d pose = kfs->at(i)->pose_est.matrix();
    poses.push_back(pose);

    // Iterating through each frame
    for (size_t j = 0; j < kfs->at(i)->frame_poses.size(); j++) {
      // Get the frame pose
      Eigen::Isometry3d frame_pose = kfs->at(i)->frame_poses[j];

      // Transform the frame pose
      Eigen::Isometry3d corr_pose = pose_iso * frame_pose;

      // Adding the pose to the list
      poses.push_back(corr_pose.matrix());
    }
  }

  // Writing poses
  std::ofstream poses_file;
  poses_file.open(results_file.c_str(), std::ios::out | std::ios::trunc);

  for (size_t pose_ind = 0; pose_ind < poses.size(); pose_ind++) {
    for (int i = 0; i < 3; i++) { // Rows
      for (int j = 0; j < 4; j++) { // Cols
        poses_file << poses[pose_ind](i, j);

        if (j != 3) {
          poses_file << " ";
        }
      }

      if (i == 2) {
        poses_file << std::endl;
      } else {
        poses_file << " ";
      }
    }
  }  
}

int processKeyframes() {

  // Processing new Keyframes
  int kf_processed = 0;
  for (size_t i = 0; i < queue_kfs.size(); i++) {    
    lihash_slam::Keyframe* kf = queue_kfs[i];
    map->addKeyframe(kf);
  }

  queue_kfs.erase(queue_kfs.begin(), queue_kfs.end());

  return kf_processed;
}

int processLoopClosures() {

  // Processing Loop Closures
  int lc_processed = 0;
  std::deque<lihash_slam::LoopClosure> new_queue_lcs;

  for (size_t i = 0; i < queue_lcs.size(); i++) {
    
    lihash_slam::LoopClosure lc = queue_lcs[i];

    if (map->existsKeyframe(lc.f1) && map->existsKeyframe(lc.f2)) {

      // Creating an isometry
      Eigen::Quaterniond q_current(lc.rel_pose.orientation.w,
                                  lc.rel_pose.orientation.x,
                                  lc.rel_pose.orientation.y,
                                  lc.rel_pose.orientation.z);
      Eigen::Vector3d t_current(lc.rel_pose.position.x,
                                lc.rel_pose.position.y,
                                lc.rel_pose.position.z);
      Eigen::Isometry3d rel_pose = Eigen::Isometry3d::Identity();
      rel_pose.linear() = q_current.toRotationMatrix();
      rel_pose.translation() = t_current;

      map->addLoopClosure(lc.f1, lc.f2, rel_pose);

      lc_processed++;      
    } else {
      new_queue_lcs.push_back(lc);
    }
  }

  queue_lcs.erase(queue_lcs.begin(), queue_lcs.end());
  queue_lcs = new_queue_lcs;

  return lc_processed;
}

void keyframeClb(const lihash_slam::KeyframeMessageConstPtr& kf_msg) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(kf_msg->points, *pc_new);

  // Creating an isometry from the relative pose
  Eigen::Quaterniond q_current(kf_msg->rel_pose.orientation.w,
                               kf_msg->rel_pose.orientation.x,
                               kf_msg->rel_pose.orientation.y,
                               kf_msg->rel_pose.orientation.z);
  Eigen::Vector3d t_current(kf_msg->rel_pose.position.x,
                            kf_msg->rel_pose.position.y,
                            kf_msg->rel_pose.position.z);

  Eigen::Isometry3d pose_rel = Eigen::Isometry3d::Identity();
  pose_rel.linear() = q_current.toRotationMatrix();
  pose_rel.translation() = t_current;

  // Creating an isometry from the odometry pose
  Eigen::Quaterniond q_current_odom(kf_msg->odom_pose.orientation.w,
                               kf_msg->odom_pose.orientation.x,
                               kf_msg->odom_pose.orientation.y,
                               kf_msg->odom_pose.orientation.z);
  Eigen::Vector3d t_current_odom(kf_msg->odom_pose.position.x,
                            kf_msg->odom_pose.position.y,
                            kf_msg->odom_pose.position.z);  
  Eigen::Isometry3d pose_odom = Eigen::Isometry3d::Identity();
  pose_odom.linear() = q_current_odom.toRotationMatrix();
  pose_odom.translation() = t_current_odom;

  lihash_slam::Keyframe* kf = new lihash_slam::Keyframe(
                                kf_msg->header.seq,
                                pose_rel,
                                pc_new);
  kf->pose_odom = pose_odom;
  kf->stamp_ = kf_msg->header.stamp;
  kf->addFramePoses(kf_msg->rel_poses);

  queue_kfs.push_back(kf);
}

void lcClb(const lihash_slam::LoopClosureConstPtr& lc_msg) {
  
  queue_lcs.push_back(*lc_msg);  
}

void optimize(const ros::TimerEvent& event) {
  
  // Inserting new received keyframes
  processKeyframes();

  // Inserting new detected LCs
  int nloops = processLoopClosures();  

  // Optimize the graph is there any new loop closure
  if (nloops > 0) {
    map->optimize();    
  }
}

void publishTF(const ros::TimerEvent& event) {
  
  ros::Time kf_stamp = ros::Time::now() + ros::Duration(tf_period);

  // Sending the mTo transform
  if (publish_tf) {

    // Getting keyframes
    std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();

    if (kfs->size() > 0) {

      // Saving the optimized position of the last keyframe    
      Eigen::Isometry3d mTk = kfs->at(kfs->size() - 1)->pose_est;
      Eigen::Isometry3d oTk = kfs->at(kfs->size() - 1)->pose_odom;

      // Computing the correction from map to odom
      mTo = mTk * oTk.inverse();
    }

    Eigen::Quaterniond q_current(mTo.rotation());
    Eigen::Vector3d t_current = mTo.translation();
    mTo_tf.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
    tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
    mTo_tf.setRotation(q);

    static tf::TransformBroadcaster tf_broadcaster_;
    tf_broadcaster_.sendTransform(tf::StampedTransform(mTo_tf, kf_stamp, map_frame, odom_frame));    
  }
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh("~");

  // Getting params  
  nh.param("cell_xy_size", cell_xy_size, 20.0);
  ROS_INFO("Cell XY size: %.2f", cell_xy_size);

  nh.param("cell_z_size", cell_z_size, 25.0);
  ROS_INFO("Cell Z size: %.2f", cell_z_size);

  double resolution;
  nh.param("cell_resolution", resolution, 0.4);
  ROS_INFO("Cell Resolution: %.2f", resolution);

  int cell_min_points;
  nh.param("cell_min_points", cell_min_points, 25);
  ROS_INFO("Cell Min Points: %i", cell_min_points);

  // Publish TF
  nh.param("publish_tf", publish_tf, true);
  ROS_INFO("Publish TF: %s", publish_tf ? "Yes" : "No");
  
  nh.param("publish_tf_period", tf_period, 1.0);
  ROS_INFO("Publish TF Period: %.2f", tf_period);

  double map_period;
  nh.param("publish_map_period", map_period, 4.0);
  ROS_INFO("Publish Map Period: %.2f", map_period);

  double optim_period;
  nh.param("optimize_period", optim_period, 2.0);
  ROS_INFO("Optimization Period: %.2f", optim_period);

  // Map frame
  nh.param<std::string>("map_frame", map_frame, "map");
  ROS_INFO("Map frame: %s", map_frame.c_str());

  // Odom frame
  nh.param<std::string>("odom_frame", odom_frame, "odom");
  ROS_INFO("Odom frame: %s", odom_frame.c_str());

  // Save results
  nh.param("save_results", save_results, false);
  ROS_INFO("Save results: %s", save_results ? "Yes" : "No");

  std::string results_file;
  nh.param<std::string>("results_file", results_file, "/home/emilio/Escritorio/poses.txt");
  ROS_INFO("Results file: %s", results_file.c_str());

  // Reading visualization params
  nh.param("viz_kf_size", viz_kf_size, 1.25);
  ROS_INFO("RVIZ KF Size: %.2f", viz_kf_size);

  // Initializing the map
  map = new lihash_slam::Map(cell_xy_size, cell_z_size, resolution, cell_min_points);

  // Initializing transforms
  mTo = Eigen::Isometry3d::Identity();
  mTo_tf.setOrigin(tf::Vector3(0, 0, 0));
  tf::Quaternion q(0, 0, 0, 1);
  mTo_tf.setRotation(q);

  // ROS Interface
  // Keyframes
  ros::Subscriber kf_subs = nh.subscribe("kfs", 1000, keyframeClb);
  
  // LCs
  ros::Subscriber lc_sub = nh.subscribe("lc", 100, lcClb);

  // Publishers
  map_points_pub    = nh.advertise<sensor_msgs::PointCloud2>("map/points", 120, true);
  map_kfs_pub       = nh.advertise<visualization_msgs::MarkerArray>("map/keyframes", 120, true);
  map_cells_pub     = nh.advertise<visualization_msgs::Marker>("map/cells", 120, true);
  map_traj_pub      = nh.advertise<visualization_msgs::Marker>("map/trajectory", 120, true);
  map_traj_path_pub = nh.advertise<nav_msgs::Path>("map/trajectory_path", 120, true);

  // Timers  
  ros::Timer pub_timer = nh.createTimer(ros::Duration(map_period), publishMap);
  ros::Timer tf_timer  = nh.createTimer(ros::Duration(tf_period), publishTF);
  ros::Timer opt_timer = nh.createTimer(ros::Duration(optim_period), optimize);

  // Receiving messages
  ros::spin();

  // Last mapping procedure
  map->optimize();

  // Writing results to a file
  if (save_results) {
    ROS_INFO("Writing results ...");  
    writeResults(results_file); 
    ROS_INFO("Done!");
  }

  return 0;
}
 