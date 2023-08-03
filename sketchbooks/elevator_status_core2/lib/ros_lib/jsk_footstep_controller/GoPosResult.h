#ifndef _ROS_jsk_footstep_controller_GoPosResult_h
#define _ROS_jsk_footstep_controller_GoPosResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_footstep_controller
{

  class GoPosResult : public ros::Msg
  {
    public:

    GoPosResult()
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

    const char * getType(){ return "jsk_footstep_controller/GoPosResult"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
