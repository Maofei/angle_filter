/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <angle_filter/AngleFilter.h>
#include <geometry_utils/GeometryUtils.h>
#include <parameter_utils/ParameterUtils.h>

namespace gu = geometry_utils;
namespace pu = parameter_utils;

AngleFilter::AngleFilter() {}
AngleFilter::~AngleFilter() {}

bool AngleFilter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "AngleFilter");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool AngleFilter::LoadParameters(const ros::NodeHandle& n) {
  // Load normal computation parameters.
  if (!pu::Get("angle_filter/vertical_mask_min", v_angle_min_)) return false;
  if (!pu::Get("angle_filter/vertical_mask_max", v_angle_max_)) return false;
  if (!pu::Get("angle_filter/horizontal_mask_min", h_angle_min_)) return false;
  if (!pu::Get("angle_filter/horizontal_mask_max", h_angle_max_)) return false;

  return true;
}

bool AngleFilter::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  cloud_sub_ = nl.subscribe("point_cloud", 100,
                            &AngleFilter::PointCloudCallback, this);
  filtered_cloud_pub_ =
      nl.advertise<PointCloud>("filtered_point_cloud", 10, false);

  return true;
}

void AngleFilter::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  // Don't do any work if nobody is listening.
  if (filtered_cloud_pub_.getNumSubscribers() == 0)
    return;

  // Filter() automatically publishes the result, so we don't need the
  // returned point cloud.
  PointCloud::Ptr unused(new PointCloud());
  Filter(msg, unused);
}

bool AngleFilter::Filter(const PointCloud::ConstPtr& points,
                         PointCloud::Ptr filtered_points) const {
  if (filtered_points == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Copy point cloud header.
  filtered_points->header = points->header;

  // Iterate over points in the original cloud.
  for (const auto& point : points->points) {
    const gu::Vec3 p(point.x, point.y, point.z);

    // Convert to spherical coordinates. Second two elements of the return
    // vector are theta and phi (horizontal and vertical angles). Check against
    // the loaded configuration parameters to see whether we should add the
    // point to the output cloud.
    const gu::Vec3 spherical = gu::CartesianToSpherical(p);

    const double theta = spherical.Y() - 0.5f * M_PI;
    const double phi = spherical.Z();
    if (theta < v_angle_min_ ||
        theta > v_angle_max_ ||
        phi < h_angle_min_ ||
        phi > h_angle_max_) {
      filtered_points->push_back(point);
    }
  }

  // Publish the new point cloud.
  if (filtered_cloud_pub_.getNumSubscribers() > 0) {
    filtered_cloud_pub_.publish(*filtered_points);
  }

  return true;
}
