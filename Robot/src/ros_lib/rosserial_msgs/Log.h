#ifndef ros_rosserial_msgs_Log_h
#define ros_rosserial_msgs_Log_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace rosserial_msgs
{

  class Log : public ros::Msg
  {
    public:
      unsigned char level;
      char * msg;
      enum { DEBUG = 0 };
      enum { INFO = 1 };
      enum { WARN = 2 };
      enum { ERROR = 3 };
      enum { FATAL = 4 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned char real;
        unsigned char base;
      } u_level;
      u_level.real = this->level;
      *(outbuffer + offset + 0) = (u_level.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      long * length_msg = (long *)(outbuffer + offset);
      *length_msg = strlen( (const char*) this->msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, *length_msg);
      offset += *length_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned char real;
        unsigned char base;
      } u_level;
      u_level.base = 0;
      u_level.base |= ((typeof(u_level.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->level = u_level.real;
      offset += sizeof(this->level);
      uint32_t length_msg = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
     return offset;
    }

    const char * getType(){ return "rosserial_msgs/Log"; };

  };

}
#endif