#ifndef _ROS_denso_robot_core_MoveValueAction_h
#define _ROS_denso_robot_core_MoveValueAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "denso_robot_core/MoveValueActionGoal.h"
#include "denso_robot_core/MoveValueActionResult.h"
#include "denso_robot_core/MoveValueActionFeedback.h"

namespace denso_robot_core
{

  class MoveValueAction : public ros::Msg
  {
    public:
      typedef denso_robot_core::MoveValueActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef denso_robot_core::MoveValueActionResult _action_result_type;
      _action_result_type action_result;
      typedef denso_robot_core::MoveValueActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    MoveValueAction():
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

    const char * getType(){ return "denso_robot_core/MoveValueAction"; };
    const char * getMD5(){ return "13fdd32cca9a30de56c0d6ebc9a29af5"; };

  };

}
#endif
