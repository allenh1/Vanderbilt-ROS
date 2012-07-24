#ifndef ros_SERVICE_SetCameraInfo_h
#define ros_SERVICE_SetCameraInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "sensor_msgs/CameraInfo.h"

namespace sensor_msgs
{

static const char SETCAMERAINFO[] = "sensor_msgs/SetCameraInfo";

  class SetCameraInfoRequest : public ros::Msg
  {
    public:
      sensor_msgs::CameraInfo camera_info;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->camera_info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->camera_info.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETCAMERAINFO; };

  };

  class SetCameraInfoResponse : public ros::Msg
  {
    public:
      bool success;
      char * status_message;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        bool real;
        unsigned char base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      long * length_status_message = (long *)(outbuffer + offset);
      *length_status_message = strlen( (const char*) this->status_message);
      offset += 4;
      memcpy(outbuffer + offset, this->status_message, *length_status_message);
      offset += *length_status_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        unsigned char base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((typeof(u_success.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_status_message = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_message; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_status_message-1]=0;
      this->status_message = (char *)(inbuffer + offset-1);
      offset += length_status_message;
     return offset;
    }

    const char * getType(){ return SETCAMERAINFO; };

  };

}
#endif