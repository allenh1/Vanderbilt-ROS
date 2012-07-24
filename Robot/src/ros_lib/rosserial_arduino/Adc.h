#ifndef ros_rosserial_arduino_Adc_h
#define ros_rosserial_arduino_Adc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace rosserial_arduino
{

  class Adc : public ros::Msg
  {
    public:
      unsigned int adc0;
      unsigned int adc1;
      unsigned int adc2;
      unsigned int adc3;
      unsigned int adc4;
      unsigned int adc5;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned int real;
        unsigned int base;
      } u_adc0;
      u_adc0.real = this->adc0;
      *(outbuffer + offset + 0) = (u_adc0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc0.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc0);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc1;
      u_adc1.real = this->adc1;
      *(outbuffer + offset + 0) = (u_adc1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc1);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc2;
      u_adc2.real = this->adc2;
      *(outbuffer + offset + 0) = (u_adc2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc2);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc3;
      u_adc3.real = this->adc3;
      *(outbuffer + offset + 0) = (u_adc3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc3);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc4;
      u_adc4.real = this->adc4;
      *(outbuffer + offset + 0) = (u_adc4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc4);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc5;
      u_adc5.real = this->adc5;
      *(outbuffer + offset + 0) = (u_adc5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_adc5.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->adc5);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned int real;
        unsigned int base;
      } u_adc0;
      u_adc0.base = 0;
      u_adc0.base |= ((typeof(u_adc0.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc0.base |= ((typeof(u_adc0.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc0 = u_adc0.real;
      offset += sizeof(this->adc0);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc1;
      u_adc1.base = 0;
      u_adc1.base |= ((typeof(u_adc1.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc1.base |= ((typeof(u_adc1.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc1 = u_adc1.real;
      offset += sizeof(this->adc1);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc2;
      u_adc2.base = 0;
      u_adc2.base |= ((typeof(u_adc2.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc2.base |= ((typeof(u_adc2.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc2 = u_adc2.real;
      offset += sizeof(this->adc2);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc3;
      u_adc3.base = 0;
      u_adc3.base |= ((typeof(u_adc3.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc3.base |= ((typeof(u_adc3.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc3 = u_adc3.real;
      offset += sizeof(this->adc3);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc4;
      u_adc4.base = 0;
      u_adc4.base |= ((typeof(u_adc4.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc4.base |= ((typeof(u_adc4.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc4 = u_adc4.real;
      offset += sizeof(this->adc4);
      union {
        unsigned int real;
        unsigned int base;
      } u_adc5;
      u_adc5.base = 0;
      u_adc5.base |= ((typeof(u_adc5.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_adc5.base |= ((typeof(u_adc5.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->adc5 = u_adc5.real;
      offset += sizeof(this->adc5);
     return offset;
    }

    const char * getType(){ return "rosserial_arduino/Adc"; };

  };

}
#endif