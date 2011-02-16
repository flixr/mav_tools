/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010,
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of CCNY Robotics Lab nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include "laser_height_estimation/laser_height_estimation.h"

LaserHeightEstimation::LaserHeightEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting LaserHeightEstimation"); 

  initialized_  = false;
  floor_height_ = 0.0;
  prev_height_  = 0.0;

  height_to_base_msg_      = boost::make_shared<std_msgs::Float64>();
  height_to_footprint_msg_ = boost::make_shared<std_msgs::Float64>();

  // **** parameters
  
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("footprint_frame", footprint_frame_))
    footprint_frame_ = "base_footprint";
  if (!nh_private_.getParam ("min_values", min_values_))
    min_values_ = 5;
  if (!nh_private_.getParam ("max_stdev", max_stdev_))
    max_stdev_ = 0.10;
  if (!nh_private_.getParam ("max_height_jump", max_height_jump_))
    max_height_jump_ = 0.25;
  
  // **** subscribers

  scan_subscriber_ = nh_.subscribe(
    scan_topic_, 5, &LaserHeightEstimation::scanCallback, this);
  imu_subscriber_ = nh_.subscribe(
    imu_topic_, 5, &LaserHeightEstimation::imuCallback, this);

  // **** publishers

  height_to_base_publisher_ = nh_.advertise<std_msgs::Float64>(
    height_to_base_topic_, 5);

  height_to_footprint_publisher_ = nh_.advertise<std_msgs::Float64>(
    height_to_footprint_topic_, 5);
}

LaserHeightEstimation::~LaserHeightEstimation()
{
  ROS_INFO("Destroying LaserHeightEstimation"); 
}

void LaserHeightEstimation::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  imu_transform_.setRotation(
    btQuaternion(imu_msg->orientation.x, imu_msg->orientation.y,
                 imu_msg->orientation.z, imu_msg->orientation.w));
/*
  double roll, pitch, yaw;
  btMatrix3x3 m(btQuaternion(
    imu_msg->orientation.x, imu_msg->orientation.y,
    imu_msg->orientation.z, imu_msg->orientation.w));

  m.getRPY(roll, pitch, yaw);
  ROS_INFO("R, P, Y: %f, %f, %f", 
    roll * 180.0/3.14159, pitch * 180.0/3.14159, yaw * 180.0/3.14159);
*/
}

void LaserHeightEstimation::scanCallback (const sensor_msgs::LaserScanPtr& scan_msg)
{
  if (!initialized_)
  {
    // if this is the first scan, lookup the static base to laser tf
    // if tf is not available yet, skip this scan
    if (!setBaseToLaserTf(scan_msg)) return;
    initialized_ = true;
  }

  // **** get required transforms

  btTransform rotated_laser     = imu_transform_ * base_to_laser_;
  btTransform rotated_footprint = imu_transform_ * base_to_footprint_;
 
  // **** get vector of height values
  
  std::vector<double> values;
  for(unsigned int i = 0; i < scan_msg->ranges.size(); i++)
  {
    if (scan_msg->ranges[i] > scan_msg->range_min && scan_msg->ranges[i] < scan_msg->range_max)
    {
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      btVector3 v(cos(angle)*scan_msg->ranges[i], sin(angle)*scan_msg->ranges[i], 0.0);
      btVector3 p = rotated_laser * v;
      
      values.push_back(p.getZ());
    }
  }

  if (values.size() < min_values_)
  {
    ROS_WARN("Not enough valid values to determine height, skipping (%d collected, %d needed)",
      values.size(), min_values_);
    return;
  }

  // **** get mean and standard dev

  double mean_value, stdev_value;
  getStats(values, mean_value, stdev_value);

  if (stdev_value > max_stdev_)
  {
    ROS_WARN("Stdev of height readings too big to determine height, skipping (stdev is %f, max is %f)",
      stdev_value, max_stdev_);
    return;
  }

  // **** estimate height (to base and to footprint)

  double height_to_base = 0.0 - mean_value + floor_height_;
  double height_to_footprint = rotated_footprint.getOrigin().getZ() - mean_value + floor_height_; 
 
  // **** check for discontinuity
 
  double height_jump = prev_height_ - height_to_base;

  if (fabs(height_jump) > max_height_jump_)
  {
    ROS_WARN("Laser Height Estimation: Floor Discontinuity detected");
    floor_height_ += height_jump;
    height_to_base += height_jump;
    height_to_footprint += height_jump;
  }

  prev_height_ = height_to_base;

  // **** publish height message

  height_to_base_msg_->data = height_to_base;
  height_to_footprint_msg_->data = height_to_footprint;
  height_to_base_publisher_.publish(height_to_base_msg_);
  height_to_footprint_publisher_.publish(height_to_footprint_msg_);
}

bool LaserHeightEstimation::setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg)
{
  ros::Time time = scan_msg->header.stamp;

  // **** get transform base to laser

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, scan_msg->header.frame_id, time, ros::Duration(3.0));

    tf_listener_.lookupTransform (
      base_frame_, scan_msg->header.frame_id, time, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("LHE: Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }
  
  base_to_laser_ = base_to_laser_tf;

  // **** get transform base to base_footprint

  tf::StampedTransform base_to_footprint_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, footprint_frame_, time, ros::Duration(3.0));

    tf_listener_.lookupTransform (
      base_frame_, footprint_frame_, time, base_to_footprint_tf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("LHE: Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }
  
  base_to_footprint_ = base_to_footprint_tf;

  return true;
}

void LaserHeightEstimation::getStats(const std::vector<double> values, double& ave, double& stdev)
{
  double sum   = 0.0;
  double sumsq = 0.0;

  for (size_t i = 0; i < values.size(); ++i)
    sum += values[i];

  ave = sum/values.size();

  for (size_t i = 0; i < values.size(); ++i)
    sumsq += (values[i] - ave) * (values[i] - ave);

  stdev = sqrt(sumsq/values.size());
}