#ifndef MAV_MSGS_ASCTEC_COMMON_H
#define MAV_MSGS_ASCTEC_COMMON_H

namespace mav
{

// **** mav ROS namespace and topics

const std::string ROS_NAMESPACE   = "mav";

const std::string STATE_TOPIC           = "state";

const std::string POSE_TOPIC        = "pose";

const std::string LASER_ODOM_TOPIC      = "laser_odom";

const std::string HEIGHT_TO_BASE_TOPIC       = "height_to_base";
const std::string HEIGHT_TO_FOOTPRINT_TOPIC  = "height_to_footprint";

const std::string CMD_POSE_TOPIC        = "cmd_pose";
const std::string CMD_HEIGHT_TOPIC      = "cmd_height";
const std::string CMD_THRUST_TOPIC      = "cmd_thrust";
const std::string CMD_YAW_TOPIC         = "cmd_yaw";

const std::string IMU_TOPIC               = "imu";
const std::string P_HEIGHT_TOPIC          = "pressure_height";
const std::string P_HEIGHT_FILTERED_TOPIC = "pressure_height_filtered";

// **** conversion units

const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

// from asctec CtrlInput definitions
const double ROS_TO_ASC_THRUST = 4095.0;      // converts from [ 0, 1] to thrust counts
const double ROS_TO_ASC_YAW    = 2047.0;      // converts from [-1, 1] to yaw counts

enum MAVState {OFF = 0, IDLE = 1, FLYING = 2};

};



#endif // MAV_MSGS_ASCTEC_COMMON_H
