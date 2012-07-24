#ifndef ros_std_msgs_MultiArrayDimension_h
#define ros_std_msgs_MultiArrayDimension_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace std_msgs
{

  class MultiArrayDimension : public ros::Msg
  {
    public:
      char * label;
      unsigned long size;
      unsigned long stride;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_label = (long *)(outbuffer + offset);
      *length_label = strlen( (const char*) this->label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, *length_label);
      offset += *length_label;
      union {
        unsigned long real;
        unsigned long base;
      } u_size;
      u_size.real = this->size;
      *(outbuffer + offset + 0) = (u_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size);
      union {
        unsigned long real;
        unsigned long base;
      } u_stride;
      u_stride.real = this->stride;
      *(outbuffer + offset + 0) = (u_stride.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stride.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stride.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stride.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stride);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_label = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      union {
        unsigned long real;
        unsigned long base;
      } u_size;
      u_size.base = 0;
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->size = u_size.real;
      offset += sizeof(this->size);
      union {
        unsigned long real;
        unsigned long base;
      } u_stride;
      u_stride.base = 0;
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stride = u_stride.real;
      offset += sizeof(this->stride);
     return offset;
    }

    const char * getType(){ return "std_msgs/MultiArrayDimension"; };

  };

}
#endif