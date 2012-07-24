#ifndef ros_SERVICE_GetLoggers_h
#define ros_SERVICE_GetLoggers_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "roscpp/Logger.h"

namespace roscpp
{

static const char GETLOGGERS[] = "roscpp/GetLoggers";

  class GetLoggersRequest : public ros::Msg
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

    const char * getType(){ return GETLOGGERS; };

  };

  class GetLoggersResponse : public ros::Msg
  {
    public:
      unsigned char loggers_length;
      roscpp::Logger st_loggers;
      roscpp::Logger * loggers;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      *(outbuffer + offset++) = loggers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < loggers_length; i++){
      offset += this->loggers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned char loggers_lengthT = *(inbuffer + offset++);
      if(loggers_lengthT > loggers_length)
        this->loggers = (roscpp::Logger*)realloc(this->loggers, loggers_lengthT * sizeof(roscpp::Logger));
      offset += 3;
      loggers_length = loggers_lengthT;
      for( unsigned char i = 0; i < loggers_length; i++){
      offset += this->st_loggers.deserialize(inbuffer + offset);
        memcpy( &(this->loggers[i]), &(this->st_loggers), sizeof(roscpp::Logger));
      }
     return offset;
    }

    const char * getType(){ return GETLOGGERS; };

  };

}
#endif