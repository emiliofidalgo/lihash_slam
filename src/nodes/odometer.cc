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

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// LiHash SLAM
#include <lihash_slam/defs.h>
#include <lihash_slam/lidar_odometry.h>

lihash_slam::LidarOdometer* odom;

void lidarClb(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);
  
  lihash_slam::PointCloud::Ptr pc_edges(new lihash_slam::PointCloud);
  odom->process(pc_new, lidar_msg->header);
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "odometer");
  ros::NodeHandle nh("~");

  // Creating PointCloudProcessor
  odom = new lihash_slam::LidarOdometer(nh);

  // ROS Interface
  ros::Subscriber pc_subs = nh.subscribe("points", 1000, lidarClb);

  // Receiving messages
  ros::spin();

  return 0;
}
