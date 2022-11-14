#!/bin/bash

# This script is used to generate a reconstruction from the output of the SLAM system.

# Variables
POSE_DIR=$1
BAG_DIR=$2
PCD_DIR="$POSE_DIR/pcds/"

# Starting reconstruction
echo "Starting reconstruction"
echo "SLAM Directory: $POSE_DIR"
echo "Bag Directory: $BAG_DIR"

# Check if the pcds directory exists
if [ ! -d "$PCD_DIR" ]; then
    mkdir -p $PCD_DIR
    # Extracting point clouds from the bag file
    echo "Extracting point clouds from the bag file ..."
    rosrun pcl_ros bag_to_pcd $BAG_DIR/pc*.bag /os_cloud/points $PCD_DIR
fi

# Calling the reconstruction script
echo "Performing the reconstruction ..."
roslaunch lihash_slam lihash_slam_reconstruct.launch input_dir:=$POSE_DIR