#ifndef ros_SERVICE_GetMap_h
#define ros_SERVICE_GetMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "nav_msgs/OccupancyGrid.h"

namespace nav_msgs
{

static const char GETMAP[] = "nav_msgs/GetMap";

  class GetMapRequest : public ros::Msg
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

    const char * getType(){ return GETMAP; };

  };

  class GetMapResponse : public ros::Msg
  {
    public:
      nav_msgs::OccupancyGrid map;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETMAP; };

  };

}
#endif