#ifndef ros_std_msgs_Empty_h
#define ros_std_msgs_Empty_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace std_msgs
{

  class Empty : public ros::Msg
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

    const char * getType(){ return "std_msgs/Empty"; };

  };

}
#endif