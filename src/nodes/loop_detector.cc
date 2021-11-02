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
#include <lihash_slam/loop_detector_pose.h>
#include <lihash_slam/loop_detector_isc.h>
#include <lihash_slam/LoopClosure.h>

lihash_slam::LoopDetector* ldet;

ros::Publisher lc_pub_;

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

  ldet->addFrame(pose_msg->header.seq, pose, pc_new);

  lihash_slam::Loop loop;
  if (ldet->detect(loop)) {
    lihash_slam::LoopClosure lc_msg;
    lc_msg.header = pose_msg->header;
    lc_msg.f1 = loop.frame1;
    lc_msg.f2 = loop.frame2;
    Eigen::Quaterniond q_current(loop.rel_pose.rotation());
    Eigen::Vector3d t_current = loop.rel_pose.translation();    
    lc_msg.rel_pose.orientation.x = q_current.x();
    lc_msg.rel_pose.orientation.y = q_current.y();
    lc_msg.rel_pose.orientation.z = q_current.z();
    lc_msg.rel_pose.orientation.w = q_current.w();
    lc_msg.rel_pose.position.x = t_current.x();
    lc_msg.rel_pose.position.y = t_current.y();
    lc_msg.rel_pose.position.z = t_current.z();
    lc_pub_.publish(lc_msg);
  }
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "loop_detector");
  ros::NodeHandle nh("~");

  // Checking the LCD method
  int lcd_method;
  nh.param("lcd_method", lcd_method, 0);
  ROS_INFO("LCD Method: %d", lcd_method);

  // Creating the LoopDetector
  if (lcd_method == 0) {
    ldet = new lihash_slam::LoopDetectorPose();
  } else if (lcd_method == 1) {
    ldet = new lihash_slam::LoopDetectorISC();
  } else {
    ROS_ERROR("Unknown LCD method");
    return 0;
  }

  lc_pub_ = nh.advertise<lihash_slam::LoopClosure>("lc", 10);

  ldet->readParams(nh);
  ldet->init();

  // ROS Interface
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, "pose", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "points", 100);
  message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> sync(pose_sub, points_sub, 100);
  sync.registerCallback(boost::bind(&syncClb, _1, _2));

  // Receiving messages
  ros::spin();

  return 0;
}
