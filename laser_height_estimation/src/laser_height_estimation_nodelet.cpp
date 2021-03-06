/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010,
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
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

#include "laser_height_estimation/laser_height_estimation_nodelet.h"

namespace mav
{

LaserHeightEstimationNodelet::LaserHeightEstimationNodelet():
  initialized_(false),
  floor_height_(0.0),
  prev_height_(0.0)
{
}


void mav::LaserHeightEstimationNodelet::onInit ()
{
  NODELET_INFO("Initializing LaserHeightEstimation Nodelet");

  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getNodeHandle();
  ros::NodeHandle nh_private = getPrivateNodeHandle();
  ros::NodeHandle nh_mav (nh, mav::ROS_NAMESPACE);

  // **** parameters
  nh_private.param("fixed_frame", world_frame_, std::string("/world"));
  nh_private.param("base_frame", base_frame_, std::string("base_link"));
  nh_private.param("footprint_frame", footprint_frame_, std::string("base_footprint"));
  nh_private.param("min_values", min_values_, 5);
  if (min_values_ < 1)
    min_values_ = 5;
  nh_private.param("max_stdev", max_stdev_, 0.1);
  nh_private.param("max_height_jump", max_height_jump_, 0.25);
  nh_private.param("use_imu", use_imu_, true);

  // **** subscribers

  scan_subscriber_ = nh.subscribe(scan_topic_, 5, &LaserHeightEstimationNodelet::scanCallback, this);
  if (use_imu_)
  {
    imu_subscriber_ = nh.subscribe(mav::IMU_TOPIC, 5, &LaserHeightEstimationNodelet::imuCallback, this);
  }

  // **** publishers

  height_to_base_publisher_ = nh_mav.advertise<mav_msgs::Height> (mav::HEIGHT_TO_BASE_TOPIC, 5);

  height_to_footprint_publisher_ = nh_mav.advertise<mav_msgs::Height> (mav::HEIGHT_TO_FOOTPRINT_TOPIC, 5);
}


void LaserHeightEstimationNodelet::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  latest_imu_msg_ = *imu_msg;

/*
  double roll, pitch, yaw;
  btMatrix3x3 m(btQuaternion(
    imu_msg->orientation.x, imu_msg->orientation.y,
    imu_msg->orientation.z, imu_msg->orientation.w));

  m.getRPY(roll, pitch, yaw);
  NODELET_INFO("R, P, Y: %f, %f, %f",
    roll * 180.0/3.14159, pitch * 180.0/3.14159, yaw * 180.0/3.14159);
*/
}

void LaserHeightEstimationNodelet::scanCallback (const sensor_msgs::LaserScanPtr& scan_msg)
{
  if (!initialized_)
  {
    // if this is the first scan, lookup the static base to laser tf
    // if tf is not available yet, skip this scan
    if (!setBaseToLaserTf(scan_msg)) return;

    last_update_time_ = scan_msg->header.stamp;
  }

  // **** get required transforms

  if(use_imu_)
  {
    world_to_base_.setIdentity();
    btQuaternion q;
    tf::quaternionMsgToTF(latest_imu_msg_.orientation, q);
    world_to_base_.setRotation(q);
  }
  else
  {
    tf::StampedTransform world_to_base_tf;
    world_to_base_.setIdentity();
    try
    {
      tf_listener_.waitForTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, ros::Duration(0.5));
      tf_listener_.lookupTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, world_to_base_tf);
    }
    catch (tf::TransformException ex)
    {
      // transform unavailable - skip scan
      NODELET_WARN ("Skipping scan (%s)", ex.what ());
      return;
    }
    world_to_base_.setRotation( world_to_base_tf.getRotation());
  }

  btTransform rotated_laser     = world_to_base_ * base_to_laser_;
  btTransform rotated_footprint = world_to_base_ * base_to_footprint_;

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

  if (values.size() < (unsigned int) min_values_)
  {
    NODELET_WARN("Not enough valid values to determine height, skipping (%d collected, %d needed)",
        values.size(), min_values_);
    return;
  }

  // **** get mean and standard dev

  double mean_value, stdev_value;
  getStats(values, mean_value, stdev_value);

  if (stdev_value > max_stdev_)
  {
    NODELET_WARN("Stdev of height readings too big to determine height, skipping (stdev is %f, max is %f)",
        stdev_value, max_stdev_);
    return;
  }

  // **** estimate height (to base and to footprint)

  double height_to_base = 0.0 - mean_value + floor_height_;
  double height_to_footprint = rotated_footprint.getOrigin().getZ() - mean_value + floor_height_;

  // **** check for discontinuity

  double height_jump = prev_height_ - height_to_base;

  // only set initialized to true after prev_height_ is set to prevent waring about discontinuity at startup
  if (!initialized_)
  {
    prev_height_ = height_to_base;
    initialized_ = true;
    return;
  }

  if (fabs(height_jump) > max_height_jump_)
  {
    NODELET_WARN("Laser Height Estimation: Floor Discontinuity detected: %f m", height_jump);
    floor_height_ += height_jump;
    height_to_base += height_jump;
    height_to_footprint += height_jump;
  }

  double dt = (scan_msg->header.stamp - last_update_time_).toSec();

  double climb = (height_to_base - prev_height_)/dt;
  prev_height_ = height_to_base;
  last_update_time_ = scan_msg->header.stamp;

  // **** publish height message

  mav_msgs::HeightPtr height_to_base_msg;
  height_to_base_msg = boost::make_shared<mav_msgs::Height>();
  height_to_base_msg->height = height_to_base;
  height_to_base_msg->distance = height_to_base - floor_height_;
  height_to_base_msg->height_variance = stdev_value;
  height_to_base_msg->climb = climb;
  height_to_base_msg->header.stamp = scan_msg->header.stamp;
  height_to_base_msg->header.frame_id = scan_msg->header.frame_id;

  mav_msgs::HeightPtr height_to_footprint_msg;
  height_to_footprint_msg = boost::make_shared<mav_msgs::Height>();
  height_to_footprint_msg->height = height_to_footprint;
  height_to_footprint_msg->distance = height_to_footprint - floor_height_;
  height_to_footprint_msg->height_variance = stdev_value;
  height_to_footprint_msg->climb = climb;
  height_to_footprint_msg->header.stamp = scan_msg->header.stamp;
  height_to_footprint_msg->header.frame_id = scan_msg->header.frame_id;

  height_to_base_publisher_.publish(height_to_base_msg);
  height_to_footprint_publisher_.publish(height_to_footprint_msg);
}

bool LaserHeightEstimationNodelet::setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg)
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
    NODELET_WARN("Transform unavailable, skipping scan (%s)", ex.what());
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
    NODELET_WARN("Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }

  base_to_footprint_ = base_to_footprint_tf;

  return true;
}

void LaserHeightEstimationNodelet::getStats(const std::vector<double> values, double& ave, double& stdev)
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
};// namespace mav


#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (
  laser_height_estimation, LaserHeightEstimationNodelet,
  mav::LaserHeightEstimationNodelet, nodelet::Nodelet);
