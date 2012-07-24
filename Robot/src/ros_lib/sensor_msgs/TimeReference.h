#ifndef ros_sensor_msgs_TimeReference_h
#define ros_sensor_msgs_TimeReference_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace sensor_msgs
{

  class TimeReference : public ros::Msg
  {
    public:
      std_msgs::Header header;
      ros::Time time_ref;
      char * source;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.real = this->time_ref.sec;
      *(outbuffer + offset + 0) = (u_sec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_ref.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.real = this->time_ref.nsec;
      *(outbuffer + offset + 0) = (u_nsec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nsec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nsec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nsec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_ref.nsec);
      long * length_source = (long *)(outbuffer + offset);
      *length_source = strlen( (const char*) this->source);
      offset += 4;
      memcpy(outbuffer + offset, this->source, *length_source);
      offset += *length_source;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.base = 0;
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time_ref.sec = u_sec.real;
      offset += sizeof(this->time_ref.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.base = 0;
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->time_ref.nsec = u_nsec.real;
      offset += sizeof(this->time_ref.nsec);
      uint32_t length_source = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_source; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_source-1]=0;
      this->source = (char *)(inbuffer + offset-1);
      offset += length_source;
     return offset;
    }

    const char * getType(){ return "sensor_msgs/TimeReference"; };

  };

}
#endif