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

#ifndef INCLUDE_LIHASH_SLAM_DEFS_H
#define INCLUDE_LIHASH_SLAM_DEFS_H

#include <chrono>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

namespace lihash_slam {

// PCL typedefs
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

// Chrono typedefs
typedef std::chrono::high_resolution_clock Clock;

} // namespace lihash_slam

#endif // INCLUDE_LIHASH_SLAM_DEFS_H