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

#include <lihash_slam/cell.h>

namespace lihash_slam {

Cell::Cell() :
  points_(new PointCloud),
  modified_(false) {
    // TODO Prereserve memory according to a maximum number of points.
}

Cell::Cell(const PointCloud::Ptr& pc) :
  points_(new PointCloud),
  modified_(true) {
    // TODO Prereserve memory according to a maximum number of points.
    *points_ += *pc;
}

Cell::~Cell() {
  points_->clear();
}

void Cell::addPoint(const Point& p) {
  points_->push_back(p);
  modified_ = true;
}

void Cell::addPoints(const PointCloud::Ptr& pc) {
  *points_ += *pc;
  modified_ = true;
}

void Cell::clear() {
  points_->clear();
  modified_ = true;
}

void Cell::filter(pcl::VoxelGrid<Point>& filt) {
  if (points_->size() > 0) {
    filt.setInputCloud(points_);  
    filt.filter(*points_);
  }
  modified_ = false;
}

bool Cell::modified() {
  return modified_;
}

PointCloud::Ptr Cell::getPoints() {
  return points_;
}

}  // namespace lihash_slam
