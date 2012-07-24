#ifndef ros_SERVICE_Empty_h
#define ros_SERVICE_Empty_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace roscpp
{

static const char EMPTY[] = "roscpp/Empty";

  class EmptyRequest : public ros::Msg
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

    const char * getType(){ return EMPTY; };

  };

  class EmptyResponse : public ros::Msg
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

    const char * getType(){ return EMPTY; };

  };

}
#endif