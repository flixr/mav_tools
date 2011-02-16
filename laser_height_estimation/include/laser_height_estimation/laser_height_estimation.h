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

#ifndef LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H
#define LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>

const std::string scan_topic_                = "scan";
const std::string height_to_base_topic_      = "height_to_base";
const std::string height_to_footprint_topic_ = "height_to_footprint";
const std::string imu_topic_                 = "imu";

class LaserHeightEstimation
{
  private:

    // **** ros-related variables

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher  height_to_base_publisher_;
    ros::Publisher  height_to_footprint_publisher_;
    tf::TransformListener tf_listener_;

    // **** state variables

    bool initialized_;
    double floor_height_;
    double prev_height_;

    btTransform base_to_laser_;
    btTransform base_to_footprint_;
    btTransform imu_transform_;
   
    std_msgs::Float64Ptr height_to_base_msg_;
    std_msgs::Float64Ptr height_to_footprint_msg_;

    // **** parameters
  
    std::string base_frame_;
    std::string footprint_frame_;
    int min_values_;
    double max_stdev_;
    double max_height_jump_;

    // **** member functions

    void scanCallback (const sensor_msgs::LaserScanPtr& scan_msg);
    void imuCallback  (const sensor_msgs::ImuPtr&       imu_msg);
    bool setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg);
    void getStats(const std::vector<double> values, double& ave, double& stdev);

  public:
  
    LaserHeightEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LaserHeightEstimation();
};

#endif // LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H
