#ifndef ros_SERVICE_SetLoggerLevel_h
#define ros_SERVICE_SetLoggerLevel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace roscpp
{

static const char SETLOGGERLEVEL[] = "roscpp/SetLoggerLevel";

  class SetLoggerLevelRequest : public ros::Msg
  {
    public:
      char * logger;
      char * level;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_logger = (long *)(outbuffer + offset);
      *length_logger = strlen( (const char*) this->logger);
      offset += 4;
      memcpy(outbuffer + offset, this->logger, *length_logger);
      offset += *length_logger;
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
      uint32_t length_logger = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_logger; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_logger-1]=0;
      this->logger = (char *)(inbuffer + offset-1);
      offset += length_logger;
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

    const char * getType(){ return SETLOGGERLEVEL; };

  };

  class SetLoggerLevelResponse : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETLOGGERLEVEL; };

  };

}
#endif