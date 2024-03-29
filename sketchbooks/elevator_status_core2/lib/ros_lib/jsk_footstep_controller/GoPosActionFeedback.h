#ifndef _ROS_jsk_footstep_controller_GoPosActionFeedback_h
#define _ROS_jsk_footstep_controller_GoPosActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "jsk_footstep_controller/GoPosFeedback.h"

namespace jsk_footstep_controller
{

  class GoPosActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef jsk_footstep_controller::GoPosFeedback _feedback_type;
      _feedback_type feedback;

    GoPosActionFeedback():
      header(),
      status(),
      feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "jsk_footstep_controller/GoPosActionFeedback"; };
    const char * getMD5(){ return "f94482fa4ac6ec4eba7904ad5f096d59"; };

  };

}
#endif
