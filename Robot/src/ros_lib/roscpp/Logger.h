#ifndef ros_roscpp_Logger_h
#define ros_roscpp_Logger_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace roscpp
{

  class Logger : public ros::Msg
  {
    public:
      char * name;
      char * level;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_name = (long *)(outbuffer + offset);
      *length_name = strlen( (const char*) this->name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, *length_name);
      offset += *length_name;
      long * length_level = (long *)(outbuffer + offset);
      *length_level = strlen( (const char*) this->level);
      offset += 4;
      memcpy(outbuffer + offset, this->level, *length_level);
      offset += *length_level;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_level = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_level; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_level-1]=0;
      this->level = (char *)(inbuffer + offset-1);
      offset += length_level;
     return offset;
    }

    const char * getType(){ return "roscpp/Logger"; };

  };

}
#endif