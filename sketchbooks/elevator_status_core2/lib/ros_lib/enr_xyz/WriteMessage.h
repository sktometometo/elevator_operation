#ifndef _ROS_SERVICE_WriteMessage_h
#define _ROS_SERVICE_WriteMessage_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace enr_xyz
{

static const char WRITEMESSAGE[] = "enr_xyz/WriteMessage";

  class WriteMessageRequest : public ros::Msg
  {
    public:
      typedef const char* _message_type;
      _message_type message;
      typedef ros::Duration _timeout_duration_type;
      _timeout_duration_type timeout_duration;

    WriteMessageRequest():
      message(""),
      timeout_duration()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      *(outbuffer + offset + 0) = (this->timeout_duration.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout_duration.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout_duration.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout_duration.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout_duration.sec);
      *(outbuffer + offset + 0) = (this->timeout_duration.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout_duration.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout_duration.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout_duration.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout_duration.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      this->timeout_duration.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout_duration.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout_duration.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout_duration.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout_duration.sec);
      this->timeout_duration.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout_duration.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout_duration.nsec);
     return offset;
    }

    const char * getType(){ return WRITEMESSAGE; };
    const char * getMD5(){ return "16679bb19e44df036208433fb182882b"; };

  };

  class WriteMessageResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;

    WriteMessageResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    const char * getType(){ return WRITEMESSAGE; };
    const char * getMD5(){ return "937c9679a518e3a18d831e57125ea522"; };

  };

  class WriteMessage {
    public:
    typedef WriteMessageRequest Request;
    typedef WriteMessageResponse Response;
  };

}
#endif
