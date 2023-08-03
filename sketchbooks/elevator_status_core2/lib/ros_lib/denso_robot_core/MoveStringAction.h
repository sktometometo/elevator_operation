#ifndef _ROS_denso_robot_core_MoveStringAction_h
#define _ROS_denso_robot_core_MoveStringAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "denso_robot_core/MoveStringActionGoal.h"
#include "denso_robot_core/MoveStringActionResult.h"
#include "denso_robot_core/MoveStringActionFeedback.h"

namespace denso_robot_core
{

  class MoveStringAction : public ros::Msg
  {
    public:
      typedef denso_robot_core::MoveStringActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef denso_robot_core::MoveStringActionResult _action_result_type;
      _action_result_type action_result;
      typedef denso_robot_core::MoveStringActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    MoveStringAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "denso_robot_core/MoveStringAction"; };
    const char * getMD5(){ return "ddccd6f419ccc28c54b8006139e80041"; };

  };

}
#endif
