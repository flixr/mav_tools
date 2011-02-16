/* Auto-generated by genmsg_cpp for file /home/idryanov/ros/ccny-ros-pkg/ccny_experimental/mav_msgs/msg/State.msg */
#ifndef MAV_MSGS_MESSAGE_STATE_H
#define MAV_MSGS_MESSAGE_STATE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "std_msgs/Header.h"

namespace mav_msgs
{
template <class ContainerAllocator>
struct State_ : public ros::Message
{
  typedef State_<ContainerAllocator> Type;

  State_()
  : header()
  , state(0)
  {
  }

  State_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , state(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef uint8_t _state_type;
  uint8_t state;


private:
  static const char* __s_getDataType_() { return "mav_msgs/State"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "da95463f5af1db345e11ec4ce9b23d3f"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
\n\
uint8 state\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, state);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, state);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(state);
    return size;
  }

  typedef boost::shared_ptr< ::mav_msgs::State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mav_msgs::State_<ContainerAllocator>  const> ConstPtr;
}; // struct State
typedef  ::mav_msgs::State_<std::allocator<void> > State;

typedef boost::shared_ptr< ::mav_msgs::State> StatePtr;
typedef boost::shared_ptr< ::mav_msgs::State const> StateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::mav_msgs::State_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::mav_msgs::State_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace mav_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::mav_msgs::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "da95463f5af1db345e11ec4ce9b23d3f";
  }

  static const char* value(const  ::mav_msgs::State_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xda95463f5af1db34ULL;
  static const uint64_t static_value2 = 0x5e11ec4ce9b23d3fULL;
};

template<class ContainerAllocator>
struct DataType< ::mav_msgs::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mav_msgs/State";
  }

  static const char* value(const  ::mav_msgs::State_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mav_msgs::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
uint8 state\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::mav_msgs::State_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::mav_msgs::State_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mav_msgs::State_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.state);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct State_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mav_msgs::State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::mav_msgs::State_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};


} // namespace message_operations
} // namespace ros

#endif // MAV_MSGS_MESSAGE_STATE_H

