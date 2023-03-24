#ifndef _ROS_dwm1000_BeaconDataArray_h
#define _ROS_dwm1000_BeaconDataArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dwm1000/BeaconData.h"

namespace dwm1000
{

  class BeaconDataArray : public ros::Msg
  {
    public:
      uint32_t beacons_length;
      typedef dwm1000::BeaconData _beacons_type;
      _beacons_type st_beacons;
      _beacons_type * beacons;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->beacons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->beacons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->beacons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->beacons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->beacons_length);
      for( uint32_t i = 0; i < beacons_length; i++){
      offset += this->beacons[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t beacons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      beacons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      beacons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      beacons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->beacons_length);
      if(beacons_lengthT > beacons_length)
        this->beacons = (dwm1000::BeaconData*)realloc(this->beacons, beacons_lengthT * sizeof(dwm1000::BeaconData));
      beacons_length = beacons_lengthT;
      for( uint32_t i = 0; i < beacons_length; i++){
      offset += this->st_beacons.deserialize(inbuffer + offset);
        memcpy( &(this->beacons[i]), &(this->st_beacons), sizeof(dwm1000::BeaconData));
      }
     return offset;
    }

    const char * getType() { return "dwm1000_msgs/BeaconDataArray"; };
    const char * getMD5() { return "8773c386b9957fd17c8b97c17eb896e5"; };

  };

}
#endif
