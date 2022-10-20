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

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// LiHash SLAM
#include <lihash_slam/defs.h>
#include <lihash_slam/keyframe_maker.h>

lihash_slam::KeyframeMaker* kfmaker;

void syncClb(const sensor_msgs::PointCloud2ConstPtr& edges, const nav_msgs::OdometryConstPtr& pose) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(*edges, *pc_new);

  // Creating an isometry
  Eigen::Quaterniond q_current(pose->pose.pose.orientation.w,
                               pose->pose.pose.orientation.x,
                               pose->pose.pose.orientation.y,
                               pose->pose.pose.orientation.z);
  Eigen::Vector3d t_current(pose->pose.pose.position.x,
                            pose->pose.pose.position.y,
                            pose->pose.pose.position.z);

  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.linear() = q_current.toRotationMatrix();
  p.translation() = t_current;
  
  kfmaker->process(edges->header, p, pc_new);
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "kfmaker");
  ros::NodeHandle nh("~");

  // Creating Keyframe maker
  kfmaker = new lihash_slam::KeyframeMaker(nh);

  // ROS Interface
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "points", 50);
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub(nh, "pose", 50);
  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, nav_msgs::Odometry> sync(points_sub, pose_sub, 10);
  sync.registerCallback(boost::bind(&syncClb, _1, _2));

  // Receiving messages
  ros::spin();

  return 0;
}
