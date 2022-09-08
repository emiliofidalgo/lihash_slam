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

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// LiHash SLAM
#include <lihash_slam/defs.h>
#include <lihash_slam/KeyframeMessage.h>
#include <lihash_slam/loop_detector_pose.h>
#include <lihash_slam/loop_detector_isc.h>
#include <lihash_slam/loop_detector_sc.h>
#include <lihash_slam/LoopClosure.h>

lihash_slam::LoopDetector* ldet;

ros::Publisher lc_pub_;

void keyframeClb(const lihash_slam::KeyframeMessageConstPtr& kf_msg) {
  
  // Converting ROS message to PCL
  lihash_slam::PointCloud::Ptr pc_new(new lihash_slam::PointCloud);
  pcl::fromROSMsg(kf_msg->points, *pc_new);

  // Creating an isometry
  Eigen::Quaterniond q_current(kf_msg->odom_pose.orientation.w,
                               kf_msg->odom_pose.orientation.x,
                               kf_msg->odom_pose.orientation.y,
                               kf_msg->odom_pose.orientation.z);
  Eigen::Vector3d t_current(kf_msg->odom_pose.position.x,
                            kf_msg->odom_pose.position.y,
                            kf_msg->odom_pose.position.z);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q_current.toRotationMatrix();
  pose.translation() = t_current;

  ldet->addFrame(kf_msg->header.seq, pose, pc_new);

  lihash_slam::Loop loop;
  if (ldet->detect(loop)) {
    lihash_slam::LoopClosure lc_msg;
    lc_msg.header = kf_msg->header;
    lc_msg.header.frame_id = "loop";
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
  } else if (lcd_method == 2) {
    ldet = new lihash_slam::LoopDetectorSC();
  } else {
    ROS_ERROR("Unknown LCD method");
    return 0;
  }

  lc_pub_ = nh.advertise<lihash_slam::LoopClosure>("lc", 10);

  ldet->readParams(nh);
  ldet->init();

  // ROS Interface
  ros::Subscriber kf_subs = nh.subscribe("kfs", 1000, keyframeClb);

  // Receiving messages
  ros::spin();

  return 0;
}
