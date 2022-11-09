#!/bin/bash

# This script is used to generate a reconstruction from the output of the SLAM system.

# Variables
CURRENT_DIR=$1
BAG_DIR=$2
PCD_DIR="$CURRENT_DIR/pcds/"

# Starting reconstruction
echo "Starting reconstruction"
echo "SLAM Directory: $CURRENT_DIR"
echo "Bag Directory: $BAG_DIR"

# Check if the pcds directory exists
if [ ! -d "$PCD_DIR" ]; then
    mkdir -p $PCD_DIR
    # Extracting point clouds from the bag file
    echo "Extracting point clouds from the bag file ..."
    rosrun pcl_ros bag_to_pcd $BAG_DIR/*.bag /os_cloud/points $PCD_DIR
fi

# Calling the reconstruction script
echo "Performing the reconstruction ..."
roslauch lihash_slam reconstruct.launch input_dir:=$CURRENT_DIR