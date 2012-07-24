#ifndef ros_geometry_msgs_PolygonStamped_h
#define ros_geometry_msgs_PolygonStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Polygon.h"

namespace geometry_msgs
{

  class PolygonStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Polygon polygon;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->polygon.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->polygon.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/PolygonStamped"; };

  };

}
#endif