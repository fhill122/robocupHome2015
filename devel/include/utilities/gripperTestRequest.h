/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *  * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 * Auto-generated by genmsg_cpp from file /home/robocuphome/robocuphome2015/src/utilities/msg/gripperTestRequest.msg
 *
 */


#ifndef UTILITIES_MESSAGE_GRIPPERTESTREQUEST_H
#define UTILITIES_MESSAGE_GRIPPERTESTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace utilities
{
template <class ContainerAllocator>
struct gripperTestRequest_
{
  typedef gripperTestRequest_<ContainerAllocator> Type;

  gripperTestRequest_()
    : header()
    , cmd(0)
    , limb(0)  {
    }
  gripperTestRequest_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cmd(0)
    , limb(0)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int8_t _cmd_type;
  _cmd_type cmd;

   typedef int8_t _limb_type;
  _limb_type limb;


    enum { CMD_OPEN = 0 };
     enum { CMD_CLOSE = 1 };
     enum { LEFT = 3 };
     enum { RIGHT = 4 };
     enum { BOTH = 5 };
 

  typedef boost::shared_ptr< ::utilities::gripperTestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::utilities::gripperTestRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct gripperTestRequest_

typedef ::utilities::gripperTestRequest_<std::allocator<void> > gripperTestRequest;

typedef boost::shared_ptr< ::utilities::gripperTestRequest > gripperTestRequestPtr;
typedef boost::shared_ptr< ::utilities::gripperTestRequest const> gripperTestRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::utilities::gripperTestRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::utilities::gripperTestRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace utilities

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/cmake/../msg'], 'utilities': ['/home/robocuphome/robocuphome2015/src/utilities/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::utilities::gripperTestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::utilities::gripperTestRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::utilities::gripperTestRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::utilities::gripperTestRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::utilities::gripperTestRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::utilities::gripperTestRequest_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::utilities::gripperTestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5ced861508a63eeda00fb12930a63d88";
  }

  static const char* value(const ::utilities::gripperTestRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5ced861508a63eedULL;
  static const uint64_t static_value2 = 0xa00fb12930a63d88ULL;
};

template<class ContainerAllocator>
struct DataType< ::utilities::gripperTestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "utilities/gripperTestRequest";
  }

  static const char* value(const ::utilities::gripperTestRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::utilities::gripperTestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
#Request types\n\
	int8 CMD_OPEN = 0\n\
	int8 CMD_CLOSE = 1\n\
	int8 LEFT = 3\n\
	int8 RIGHT = 4\n\
	int8 BOTH = 5\n\
	\n\
int8 cmd\n\
int8 limb\n\
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
";
  }

  static const char* value(const ::utilities::gripperTestRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::utilities::gripperTestRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cmd);
      stream.next(m.limb);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct gripperTestRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::utilities::gripperTestRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::utilities::gripperTestRequest_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cmd: ";
    Printer<int8_t>::stream(s, indent + "  ", v.cmd);
    s << indent << "limb: ";
    Printer<int8_t>::stream(s, indent + "  ", v.limb);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UTILITIES_MESSAGE_GRIPPERTESTREQUEST_H
