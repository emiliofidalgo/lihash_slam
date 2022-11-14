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

// C++
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <iostream>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

// LiHash SLAM
#include <lihash_slam/defs.h>

int main(int argc, char** argv) {

  // Init node
  ros::init(argc, argv, "reconstruct");
  ros::NodeHandle nh("~");

  // Define a PointCloud publisher
  ros::Publisher pub      = nh.advertise<sensor_msgs::PointCloud2>("reconstructed", 1);
  ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("path", 1);

  // Read parameters

  // Output directory
  std::string input_dir;
  nh.param<std::string>("input_dir", input_dir, "");

  // Pose file
  std::string pose_filename = input_dir + "/poses_tum.txt";

  // Point cloud directory
  std::string point_cloud_dir = input_dir + "/pcds";

  // Output directory
  std::string output_dir = input_dir + "/recons/";

  // Remove output directory if it exists and create it again
  boost::filesystem::remove_all(output_dir);
  boost::filesystem::create_directory(output_dir);

  // Voxel grid size
  double voxel_grid_size;
  nh.param<double>("voxel_grid_size", voxel_grid_size, 0.4);

  // Voxelize every N frames
  int voxel_grid_every_n_frames;
  nh.param<int>("voxel_grid_every_n_frames", voxel_grid_every_n_frames, 1);

  // Load poses in TUM format directly from a file using Isometry3D from Eigen
  std::ifstream poses_file(pose_filename);
  if (!poses_file) {
    std::cout << "Unable to open poses file: " << pose_filename << std::endl;
    return -1;
  }

  // Reading the poses
  std::vector<Eigen::Isometry3d> poses;
  std::vector<unsigned long> timestamps;
  std::string line;
  while (getline(poses_file, line)) {

    // Separate the line into tokens using the space as a delimiter
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;
    while (getline(ss, token, ' ')) {
      tokens.push_back(token);
    }

    // Get the timestamp
    unsigned long timestamp = std::stoul(tokens[0]);
    // Saving the timestamp
    timestamps.push_back(timestamp);

    // Get the pose
    double x = std::stod(tokens[1]);
    double y = std::stod(tokens[2]);
    double z = std::stod(tokens[3]);
    double qx = std::stod(tokens[4]);
    double qy = std::stod(tokens[5]);
    double qz = std::stod(tokens[6]);
    double qw = std::stod(tokens[7]);

    // Saving the pose
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(x, y, z);
    pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    poses.push_back(pose);
  } 
  poses_file.close();  

  boost::filesystem::path dir(point_cloud_dir);
  boost::filesystem::directory_iterator it(dir), eod;

  // Count the number of PCD files in the directory
  int num_pcd = 0;
  BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod)) {
    if (boost::filesystem::is_regular_file(p) && p.extension() == ".pcd") {
      num_pcd++;
    }
  }  

  // Reset the iterators to the beginning of the directory
  it = boost::filesystem::directory_iterator(dir);
  eod = boost::filesystem::directory_iterator();

  // Iterate over all PCD files in a directory
  lihash_slam::PointCloud::Ptr reconstructed(new lihash_slam::PointCloud);

  int files_processed = 0;
  BOOST_FOREACH(boost::filesystem::path const &p, std::make_pair(it, eod)) {
    if (boost::filesystem::is_regular_file(p) && p.extension() == ".pcd") {

      // Load the point cloud
      lihash_slam::PointCloud::Ptr cloud(new lihash_slam::PointCloud);
      pcl::io::loadPCDFile(p.string(), *cloud);

      // Get the timestamp of the point cloud
      std::string pcd_filename = p.filename().string();
      // Remove file extension
      std::string timestamp_secs  = pcd_filename.substr(0, pcd_filename.find_first_of("."));
      std::string timestamp_nsecs = pcd_filename.substr(pcd_filename.find_first_of(".") + 1, 9);

      // Concatenate the seconds and nanoseconds in a string
      std::string timestamp_str = timestamp_secs + timestamp_nsecs;
      unsigned long timestamp = std::stoul(timestamp_str);    

      // Find the closest pose to the point cloud
      double best_diff = 1e9;
      int best_pose = 0;
      for (size_t i = 0; i < timestamps.size(); i++) {
        double diff = std::abs(static_cast<long>(timestamp - timestamps[i]));
        if (diff < best_diff) {
          best_diff = diff;
          best_pose = i;
        }
      }

      // Filter point cloud by distance
      lihash_slam::PointCloud::Ptr filtered(new lihash_slam::PointCloud);
      for (size_t i = 0; i < cloud->points.size(); i++) {
        double dist = std::sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y + cloud->points[i].z * cloud->points[i].z);
        if (dist > 0.7) {
          filtered->points.push_back(cloud->points[i]);
        }
      }

      // Transform the point cloud
      lihash_slam::PointCloud::Ptr transformed_cloud(new lihash_slam::PointCloud);
      pcl::transformPointCloud(*filtered, *transformed_cloud, poses[best_pose].matrix());

      *reconstructed += *transformed_cloud;

      // Filter the reconstructed point cloud  
      if (files_processed % voxel_grid_every_n_frames == 0) {
        pcl::VoxelGrid<lihash_slam::Point> voxel_grid;
        voxel_grid.setInputCloud(reconstructed);
        voxel_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
        voxel_grid.filter(*reconstructed);
      }

      // Write the number of processed files from the total by console
      files_processed++;
      std::cout << "\r Files: " << files_processed << " / " << num_pcd << " -- Points: " << reconstructed->size() << std::flush;
    }    
  }

  // Print the number of points in the reconstructed point cloud
  std::cout << "\nNumber of points in the reconstructed point cloud: " << reconstructed->size() << std::endl;

  // Save the reconstructed point cloud
  std::string transformed_pcd_filename = output_dir + "reconstruction.pcd";
  pcl::io::savePCDFile(transformed_pcd_filename, *reconstructed, true);

  std::string transformed_ply_filename = output_dir + "reconstruction.ply";
  pcl::io::savePLYFile(transformed_ply_filename, *reconstructed, true);

  // Publish the reconstructed point cloud
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*reconstructed, msg);
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  pub.publish(msg);

  // Publishing the poses as a path
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  for (size_t i = 0; i < poses.size(); i++) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    // pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.position.x = poses[i].translation().x();
    pose_stamped.pose.position.y = poses[i].translation().y();
    pose_stamped.pose.position.z = poses[i].translation().z();
    pose_stamped.pose.orientation.x = Eigen::Quaterniond(poses[i].linear()).x();
    pose_stamped.pose.orientation.y = Eigen::Quaterniond(poses[i].linear()).y();
    pose_stamped.pose.orientation.z = Eigen::Quaterniond(poses[i].linear()).z();
    pose_stamped.pose.orientation.w = Eigen::Quaterniond(poses[i].linear()).w();
    path.poses.push_back(pose_stamped);
  }
  pub_path.publish(path);

  ros::spin();

  return 0;
} 