#ifndef ros_SERVICE_GetPlan_h
#define ros_SERVICE_GetPlan_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

namespace nav_msgs
{

static const char GETPLAN[] = "nav_msgs/GetPlan";

  class GetPlanRequest : public ros::Msg
  {
    public:
      geometry_msgs::PoseStamped start;
      geometry_msgs::PoseStamped goal;
      float tolerance;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->start.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      union {
        float real;
        unsigned long base;
      } u_tolerance;
      u_tolerance.real = this->tolerance;
      *(outbuffer + offset + 0) = (u_tolerance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tolerance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->start.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
      union {
        float real;
        unsigned long base;
      } u_tolerance;
      u_tolerance.base = 0;
      u_tolerance.base |= ((typeof(u_tolerance.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tolerance.base |= ((typeof(u_tolerance.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tolerance.base |= ((typeof(u_tolerance.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tolerance.base |= ((typeof(u_tolerance.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tolerance = u_tolerance.real;
      offset += sizeof(this->tolerance);
     return offset;
    }

    const char * getType(){ return GETPLAN; };

  };

  class GetPlanResponse : public ros::Msg
  {
    public:
      nav_msgs::Path plan;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->plan.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->plan.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPLAN; };

  };

}
#endif