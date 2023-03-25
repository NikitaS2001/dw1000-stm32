#pragma once

#include "ros_lib/dwm1000/BeaconData.h"
#include <vector>

void CommTask(void* pvParameters);

void CommSendBeaconData(const dwm1000::BeaconData& beaconData);
