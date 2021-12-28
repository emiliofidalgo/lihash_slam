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
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
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

// Variable to check if a LC has been added
bool lc_added;

// ROS
ros::Publisher map_points_pub;
ros::Publisher map_kfs_pub;
ros::Publisher map_cells_pub;
ros::Publisher map_traj_pub;

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

void publishKeyframes() {

  ros::Time time = ros::Time::now();  
  
  // Publishing keyframes
  if (map_kfs_pub.getNumSubscribers() > 0) {
    // Keyframes
    visualization_msgs::Marker marker_kfs;
    marker_kfs.header.frame_id = "world";
    marker_kfs.header.stamp = time;
    marker_kfs.ns = "kfs";
    marker_kfs.id = 0;
    marker_kfs.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_kfs.action = visualization_msgs::Marker::ADD;
    marker_kfs.scale.x = 1.25;
    marker_kfs.scale.y = 1.25;
    marker_kfs.scale.z = 1.25;
    marker_kfs.color.a = 0.5;
    marker_kfs.color.b = 1.0;
    marker_kfs.pose.orientation.w = 1.0;

    // Links between them
    visualization_msgs::Marker marker_links;
    marker_links.header.frame_id = "world";
    marker_links.header.stamp = time;
    marker_links.ns = "kfs";
    marker_links.id = 1;
    marker_links.type = visualization_msgs::Marker::LINE_STRIP;
    marker_links.action = visualization_msgs::Marker::ADD;
    marker_links.scale.x = 0.25;
    marker_links.color.a = 0.5;    
    marker_links.color.b = 1.0;
    marker_links.pose.orientation.w = 1.0;

    // Loops
    visualization_msgs::Marker marker_loops;
    marker_loops.header.frame_id = "world";
    marker_loops.header.stamp = time;
    marker_loops.ns = "kfs";
    marker_loops.id = 2;
    marker_loops.type = visualization_msgs::Marker::LINE_LIST;
    marker_loops.action = visualization_msgs::Marker::ADD;
    marker_loops.scale.x = 0.25;
    marker_loops.color.a = 0.5;    
    marker_loops.color.r = 1.0;
    marker_loops.pose.orientation.w = 1.0;

    std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();
    marker_kfs.points.resize(kfs->size());
    marker_links.points.resize(kfs->size());
    for (size_t i = 0; i < kfs->size(); i++) {
      Eigen::Vector3d pose = kfs->at(i)->pose.translation();
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
        pose2.x = kfs->at(cand_ind)->pose.translation().x();
        pose2.y = kfs->at(cand_ind)->pose.translation().y();
        pose2.z = kfs->at(cand_ind)->pose.translation().z();        

        marker_loops.points.push_back(pose1);
        marker_loops.points.push_back(pose2);
        marker_loops.colors.push_back(marker_loops.color);
        marker_loops.colors.push_back(marker_loops.color);
      }
    }

    // Marker array
    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker_kfs);
    markers.markers.push_back(marker_links);
    if (marker_loops.points.size() > 0) {
      markers.markers.push_back(marker_loops);
    }

    map_kfs_pub.publish(markers);
  }
}

void publishMap(const ros::TimerEvent& event) {

  ros::Time time = ros::Time::now();
  
  // Publishing the current map
  if (map_points_pub.getNumSubscribers() > 0) {
    lihash_slam::PointCloud::Ptr m = map->getMapPoints();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*m, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = time;
    map_points_pub.publish(cloud_msg);
  }

  // Publishing cells
  if (map_cells_pub.getNumSubscribers() > 0) {
    // Creating the template msg
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = time;
    marker.ns = "cells";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 20.0;
    marker.scale.y = 20.0;
    marker.scale.z = 25.0;
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
    marker_traj.header.frame_id = "world";
    marker_traj.header.stamp = time;
    marker_traj.ns = "traj";
    marker_traj.id = 0;
    marker_traj.type = visualization_msgs::Marker::LINE_STRIP;
    marker_traj.action = visualization_msgs::Marker::ADD;
    marker_traj.scale.x = 0.25;
    marker_traj.color.a = 0.5;    
    marker_traj.color.r = 1.0;
    marker_traj.color.g = 1.0;
    marker_traj.pose.orientation.w = 1.0;

    std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();    
    for (size_t i = 0; i < kfs->size(); i++) {
      // Getting the KF pose
      Eigen::Isometry3d pose = kfs->at(i)->pose;

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
}

void writeResults() {

  // Getting corrected poses
  std::vector<Eigen::Matrix4d> poses;
  std::vector<lihash_slam::Keyframe*>* kfs = map->getKeyframes();
  for (size_t i = 0; i < kfs->size(); i++) {
    // Getting the KF pose
    Eigen::Isometry3d pose_iso = kfs->at(i)->pose;
    Eigen::Matrix4d pose = kfs->at(i)->pose.matrix();
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
  std::string poses_filename = "/home/emilio/Escritorio/poses.txt";
  std::ofstream poses_file;
  poses_file.open(poses_filename.c_str(), std::ios::out | std::ios::trunc);

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

void keyframeClb(const lihash_slam::KeyframeMessageConstPtr& kf_msg) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(kf_msg->points, *pc_new);

  // Creating an isometry
  Eigen::Quaterniond q_current(kf_msg->pose.orientation.w,
                               kf_msg->pose.orientation.x,
                               kf_msg->pose.orientation.y,
                               kf_msg->pose.orientation.z);
  Eigen::Vector3d t_current(kf_msg->pose.position.x,
                            kf_msg->pose.position.y,
                            kf_msg->pose.position.z);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q_current.toRotationMatrix();
  pose.translation() = t_current;

  lihash_slam::Keyframe* kf = new lihash_slam::Keyframe(
                                kf_msg->header.seq,
                                pose,
                                pc_new);
  kf->addFramePoses(kf_msg->rel_poses);
  
  map->addKeyframe(kf);

  publishKeyframes();
}

void lcClb(const lihash_slam::LoopClosureConstPtr& lc_msg) {
  
  queue_lcs.push_back(*lc_msg);

  int nloops = processLoopClosures();

  if (nloops > 0) {
    map->optimize();    
  }
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh("~");

  // Initializing the map
  map = new lihash_slam::Map(20.0, 25.0, 0.4);
  lc_added = false;

  // ROS Interface
  // Keframes
  ros::Subscriber kf_subs = nh.subscribe("kfs", 1000, keyframeClb);
  
  // LCs
  ros::Subscriber lc_sub = nh.subscribe("lc", 100, lcClb);

  // Publishers
  map_points_pub = nh.advertise<sensor_msgs::PointCloud2>("map/points", 120, true);
  map_kfs_pub    = nh.advertise<visualization_msgs::MarkerArray>("map/keyframes", 120, true);
  map_cells_pub  = nh.advertise<visualization_msgs::Marker>("map/cells", 120, true);
  map_traj_pub   = nh.advertise<visualization_msgs::Marker>("map/trajectory", 120, true);

  // Timers
  //ros::Timer mapper_timer    = nh.createTimer(ros::Duration(2.0), mapping);
  ros::Timer pub_timer       = nh.createTimer(ros::Duration(4.0), publishMap);

  // Receiving messages
  ros::spin();

  // Last mapping procedure
  map->optimize();

  // Writing results to a file
  ROS_INFO("Writing results ...");
  writeResults();
  ROS_INFO("Done!");

  return 0;
}
 