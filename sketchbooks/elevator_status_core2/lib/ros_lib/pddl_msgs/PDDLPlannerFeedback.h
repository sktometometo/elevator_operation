#ifndef _ROS_pddl_msgs_PDDLPlannerFeedback_h
#define _ROS_pddl_msgs_PDDLPlannerFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pddl_msgs
{

  class PDDLPlannerFeedback : public ros::Msg
  {
    public:

    PDDLPlannerFeedback()
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

    const char * getType(){ return "pddl_msgs/PDDLPlannerFeedback"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif
