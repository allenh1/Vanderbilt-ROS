#ifndef ros_geometry_msgs_TransformStamped_h
#define ros_geometry_msgs_TransformStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"

namespace geometry_msgs
{

  class TransformStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      char * child_frame_id;
      geometry_msgs::Transform transform;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      long * length_child_frame_id = (long *)(outbuffer + offset);
      *length_child_frame_id = strlen( (const char*) this->child_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->child_frame_id, *length_child_frame_id);
      offset += *length_child_frame_id;
      offset += this->transform.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_child_frame_id = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_child_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_child_frame_id-1]=0;
      this->child_frame_id = (char *)(inbuffer + offset-1);
      offset += length_child_frame_id;
      offset += this->transform.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/TransformStamped"; };

  };

}
#endif