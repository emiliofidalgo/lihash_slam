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

// C++
#include <atomic>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <lihash_slam/defs.h>
#include <lihash_slam/pc_processor.h>

lihash_slam::PointCloudProcessor* pcproc;

ros::Publisher pc_edges_pub;
ros::Subscriber pc_subs;

void lidarClb(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);
  
  ROS_DEBUG("---");
  ROS_DEBUG("Initial cloud: %lu points", pc_new->points.size());
  
  lihash_slam::PointCloud::Ptr pc_edges(new lihash_slam::PointCloud);
  pcproc->process(pc_new, pc_edges);

  ROS_DEBUG("Feature extraction: %lu edges", pc_edges->points.size());

  sensor_msgs::PointCloud2 edges_msg;
  pcl::toROSMsg(*pc_edges, edges_msg);
  edges_msg.header = lidar_msg->header;
  pc_edges_pub.publish(edges_msg);
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "preprocessor");
  ros::NodeHandle nh("~");

  // Creating PointCloudProcessor
  pcproc = new lihash_slam::PointCloudProcessor(nh);

  // ROS Interface
  pc_edges_pub  = nh.advertise<sensor_msgs::PointCloud2>("edges", 10);
  pc_subs       = nh.subscribe("points", 1000, lidarClb);

  // Receiving messages
  ros::spin();

  return 0;
}
