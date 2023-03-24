#ifndef _ROS_dwm1000_BeaconData_h
#define _ROS_dwm1000_BeaconData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dwm1000
{

  class BeaconData : public ros::Msg
  {
    public:
      uint8_t id;
      float dist;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_dist;

      u_dist.real = this->dist;
      *(outbuffer + offset + 0) = (u_dist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dist.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dist.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dist.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dist);

      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_dist;
      u_dist.base = 0;
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dist.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dist = u_dist.real;
      offset += sizeof(this->dist);
     return offset;
    }

    const char * getType() { return "dwm1000_msgs/BeaconData"; };
    const char * getMD5() { return "d3e001865357c64492a9fc52f215ca3a"; };

  };

}
#endif
