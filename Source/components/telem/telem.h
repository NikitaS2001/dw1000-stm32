#pragma once

#include "ros_lib/dwm1000/BeaconData.h"
#include <vector>

void TelemTask(void* pvParameters);

void TelemSendBeaconData(const dwm1000::BeaconData& beaconData);
