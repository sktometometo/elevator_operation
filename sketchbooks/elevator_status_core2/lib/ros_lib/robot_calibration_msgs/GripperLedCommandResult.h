#ifndef _ROS_robot_calibration_msgs_GripperLedCommandResult_h
#define _ROS_robot_calibration_msgs_GripperLedCommandResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_calibration_msgs
{

  class GripperLedCommandResult : public ros::Msg
  {
    public:

    GripperLedCommandResult()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "robot_calibration_msgs/GripperLedCommandResult"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
