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

const std::string STATE_TOPIC         = "state";
const std::string POSE_TOPIC          = "pose";
const std::string VEL_X_TOPIC         = "vel_x";
const std::string VEL_Y_TOPIC         = "vel_y";
const std::string VEL_TOPIC           = "vel";
const std::string LASER_ODOM_TOPIC    = "laser_odom";
const std::string IMU_TOPIC           = "imu";
const std::string CMD_POSE_TOPIC      = "cmd_pose";
const std::string CMD_VEL_TOPIC       = "cmd_vel";
const std::string CMD_HEIGHT_TOPIC    = "cmd_height";

const std::string CMD_THRUST_TOPIC    = "cmd_thrust";
const std::string CMD_ROLL_TOPIC      = "cmd_roll";
const std::string CMD_PITCH_TOPIC     = "cmd_pitch";
const std::string CMD_YAW_RATE_TOPIC  = "cmd_yaw_rate";  

const std::string HEIGHT_TO_BASE_TOPIC       = "height_to_base";
const std::string HEIGHT_TO_FOOTPRINT_TOPIC  = "height_to_footprint";
const std::string P_HEIGHT_TOPIC             = "pressure_height";
const std::string P_HEIGHT_FILTERED_TOPIC    = "pressure_height_filtered";

enum MAVState {OFF = 0, IDLE = 1, FLYING = 2};

};

#endif // MAV_MSGS_ASCTEC_COMMON_H
