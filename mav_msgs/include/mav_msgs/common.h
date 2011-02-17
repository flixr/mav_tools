/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2010,
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
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

#ifndef MAV_MSGS_ASCTEC_COMMON_H
#define MAV_MSGS_ASCTEC_COMMON_H

namespace mav
{

// **** mav ROS namespace and topics

const std::string ROS_NAMESPACE   = "mav";

const std::string STATE_TOPIC       = "state";
const std::string POSE_TOPIC        = "pose";
const std::string LASER_ODOM_TOPIC  = "laser_odom";
const std::string IMU_TOPIC         = "imu";
const std::string CMD_POSE_TOPIC    = "cmd_pose";
const std::string CMD_HEIGHT_TOPIC  = "cmd_height";
const std::string CMD_THRUST_TOPIC  = "cmd_thrust";
const std::string CMD_ROLL_TOPIC    = "cmd_roll";
const std::string CMD_PITCH_TOPIC   = "cmd_pitch";
const std::string CMD_YAW_TOPIC     = "cmd_yaw";  
const std::string HEIGHT_TO_BASE_TOPIC       = "height_to_base";
const std::string HEIGHT_TO_FOOTPRINT_TOPIC  = "height_to_footprint";
const std::string P_HEIGHT_TOPIC             = "pressure_height";
const std::string P_HEIGHT_FILTERED_TOPIC    = "pressure_height_filtered";

// **** conversion units

const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

// from asctec CtrlInput definitions
const double ROS_TO_ASC_THRUST = 4095.0;      // converts from [ 0, 1] to thrust counts
const double ROS_TO_ASC_ROLL   = 2047.0;      // converts from [-1, 1] to roll counts
const double ROS_TO_ASC_PITCH  = 2047.0;      // converts from [-1, 1] to pitch counts
const double ROS_TO_ASC_YAW    = 2047.0;      // converts from [-1, 1] to yaw counts

enum MAVState {OFF = 0, IDLE = 1, FLYING = 2};

};

#endif // MAV_MSGS_ASCTEC_COMMON_H
