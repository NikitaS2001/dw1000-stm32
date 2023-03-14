#pragma once

#include "ros_lib/dwm1000/BeaconData.h"
#include <vector>

void CommTask(void* pvParameters);

void CommSendBeaconDataArray(const std::vector<dwm1000::BeaconData>& beaconDataArray);
