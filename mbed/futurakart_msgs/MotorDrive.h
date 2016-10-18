#ifndef _ROS_futurakart_msgs_MotorDrive_h
#define _ROS_futurakart_msgs_MotorDrive_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace futurakart_msgs
{

  class MotorDrive : public ros::Msg
  {
    public:
      float dir_pos;
      float prop_vel;

    MotorDrive():
      dir_pos(0),
      prop_vel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_dir_pos;
      u_dir_pos.real = this->dir_pos;
      *(outbuffer + offset + 0) = (u_dir_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dir_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dir_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dir_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dir_pos);
      union {
        float real;
        uint32_t base;
      } u_prop_vel;
      u_prop_vel.real = this->prop_vel;
      *(outbuffer + offset + 0) = (u_prop_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prop_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prop_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prop_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prop_vel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_dir_pos;
      u_dir_pos.base = 0;
      u_dir_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dir_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dir_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dir_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dir_pos = u_dir_pos.real;
      offset += sizeof(this->dir_pos);
      union {
        float real;
        uint32_t base;
      } u_prop_vel;
      u_prop_vel.base = 0;
      u_prop_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prop_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prop_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prop_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prop_vel = u_prop_vel.real;
      offset += sizeof(this->prop_vel);
     return offset;
    }

    const char * getType(){ return "futurakart_msgs/MotorDrive"; };
    const char * getMD5(){ return "098623011da7f8a9f7306bc9c4655711"; };

  };

}
#endif