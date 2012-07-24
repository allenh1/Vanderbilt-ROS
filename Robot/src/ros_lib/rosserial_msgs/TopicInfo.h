#ifndef ros_rosserial_msgs_TopicInfo_h
#define ros_rosserial_msgs_TopicInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace rosserial_msgs
{

  class TopicInfo : public ros::Msg
  {
    public:
      unsigned int topic_id;
      char * topic_name;
      char * message_type;
      enum { ID_PUBLISHER = 0 };
      enum { ID_SUBSCRIBER = 1 };
      enum { ID_SERVICE_SERVER = 2 };
      enum { ID_SERVICE_CLIENT = 3 };
      enum { ID_PARAMETER_REQUEST = 4 };
      enum { ID_LOG = 5 };
      enum { ID_TIME = 10 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned int real;
        unsigned int base;
      } u_topic_id;
      u_topic_id.real = this->topic_id;
      *(outbuffer + offset + 0) = (u_topic_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_topic_id.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->topic_id);
      long * length_topic_name = (long *)(outbuffer + offset);
      *length_topic_name = strlen( (const char*) this->topic_name);
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, *length_topic_name);
      offset += *length_topic_name;
      long * length_message_type = (long *)(outbuffer + offset);
      *length_message_type = strlen( (const char*) this->message_type);
      offset += 4;
      memcpy(outbuffer + offset, this->message_type, *length_message_type);
      offset += *length_message_type;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned int real;
        unsigned int base;
      } u_topic_id;
      u_topic_id.base = 0;
      u_topic_id.base |= ((typeof(u_topic_id.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_topic_id.base |= ((typeof(u_topic_id.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->topic_id = u_topic_id.real;
      offset += sizeof(this->topic_id);
      uint32_t length_topic_name = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_topic_name-1]=0;
      this->topic_name = (char *)(inbuffer + offset-1);
      offset += length_topic_name;
      uint32_t length_message_type = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message_type; ++k){
          inbuffer[k-1]=inbuffer[k];
           }
      inbuffer[offset+length_message_type-1]=0;
      this->message_type = (char *)(inbuffer + offset-1);
      offset += length_message_type;
     return offset;
    }

    const char * getType(){ return "rosserial_msgs/TopicInfo"; };

  };

}
#endif