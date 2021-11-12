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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// LiHash SLAM
#include <lihash_slam/defs.h>
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
ros::Publisher map_pub;

void syncClb(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);

  // Creating an isometry
  Eigen::Quaterniond q_current(pose_msg->pose.orientation.w,
                               pose_msg->pose.orientation.x,
                               pose_msg->pose.orientation.y,
                               pose_msg->pose.orientation.z);
  Eigen::Vector3d t_current(pose_msg->pose.position.x,
                            pose_msg->pose.position.y,
                            pose_msg->pose.position.z);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q_current.toRotationMatrix();
  pose.translation() = t_current;

  lihash_slam::Keyframe* kf = new lihash_slam::Keyframe(
                                pose_msg->header.seq,
                                pose,
                                pc_new);
  queue_kfs.push_back(kf);
}

void lcClb(const lihash_slam::LoopClosureConstPtr& lc_msg) {
  queue_lcs.push_back(*lc_msg);
}

void mapping(const ros::TimerEvent& event) {  
  
  // Processing new keyframes
  int kf_processed = 0;
  for (size_t i = 0; i < queue_kfs.size(); i++) {

    lihash_slam::Keyframe* kf = queue_kfs[i];    
    map->addKeyframe(kf);
    kf_processed++;
  }  
  queue_kfs.erase(queue_kfs.begin(), queue_kfs.end());

  // Checking if at least one KF has been added
  if (!kf_processed) return;

  // Processing Loop Closures
  int lc_processed = 0;
  for (size_t i = 0; i < queue_lcs.size(); i++) {
    
    lihash_slam::LoopClosure lc = queue_lcs[i];

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
  }
  queue_lcs.erase(queue_lcs.begin(), queue_lcs.end());

  map->optimize();
}

void publishData(const ros::TimerEvent& event) {
  
  // Publishing the current map
  if (map_pub.getNumSubscribers() > 0) {
    lihash_slam::PointCloud::Ptr m = map->getMapPoints();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*m, cloud_msg);
    cloud_msg.header.frame_id = "world";
    map_pub.publish(cloud_msg);
  }

  // TODO Publish KFs and Cells
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
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "pose", 120);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "points", 120);
  message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> sync(pose_sub, points_sub, 100);
  sync.registerCallback(boost::bind(&syncClb, _1, _2));
  
  // LCs
  ros::Subscriber lc_sub = nh.subscribe("lc", 100, lcClb);

  // Publishers
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 120, true);

  // Timers
  ros::Timer mapper_timer    = nh.createTimer(ros::Duration(2.0), mapping);
  ros::Timer pub_timer       = nh.createTimer(ros::Duration(5.0), publishData);

  // Receiving messages
  ros::spin();

  return 0;
}
