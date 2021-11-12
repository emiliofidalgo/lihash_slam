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

#include <lihash_slam/keyframe.h>

namespace lihash_slam {

Keyframe::Keyframe() :
  id(-1),
  pose(Eigen::Isometry3d::Identity()),
  points(new PointCloud),
  node(nullptr) {
    // TODO Prereserve memory according to a maximum number of points.
}

Keyframe::Keyframe(const int id_, Eigen::Isometry3d& pose_, const PointCloud::Ptr& pc_) :
  id(id_),
  pose(pose_),
  points(new PointCloud),
  node(nullptr) {
    // TODO Prereserve memory according to a maximum number of points.
    *points += *pc_;
}

Keyframe::~Keyframe() {
  points->clear();
}

}  // namespace lihash_slam
