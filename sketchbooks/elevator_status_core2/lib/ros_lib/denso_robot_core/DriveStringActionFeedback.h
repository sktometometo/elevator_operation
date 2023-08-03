#ifndef _ROS_denso_robot_core_DriveStringActionFeedback_h
#define _ROS_denso_robot_core_DriveStringActionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "denso_robot_core/DriveStringFeedback.h"

namespace denso_robot_core
{

  class DriveStringActionFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef denso_robot_core::DriveStringFeedback _feedback_type;
      _feedback_type feedback;

    DriveStringActionFeedback():
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

    const char * getType(){ return "denso_robot_core/DriveStringActionFeedback"; };
    const char * getMD5(){ return "56a4dd25baf17fe40d5ba05371b6ebe0"; };

  };

}
#endif
