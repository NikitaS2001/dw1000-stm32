#include "telem.h"

#include "shell/shell.h"

#include "ros_lib/dwm1000/BeaconDataArray.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <stdio.h>

xSemaphoreHandle s_rosSemaphore = NULL;

ros::NodeHandle s_rosNodeHandle;
dwm1000::BeaconDataArray s_beaconDataMsg;
ros::Publisher s_distPub("dwm1000/beacon_data", &s_beaconDataMsg);

std::vector<dwm1000::BeaconData> s_pendingBeaconData;

void TelemTask(void* pvParameters)
{
    SHELL_LOG("[ROS] Initializing...\r\n");

    s_rosSemaphore = xSemaphoreCreateMutex();

    configASSERT(s_rosSemaphore != NULL);
    configASSERT(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

    s_rosNodeHandle.initNode();
    s_rosNodeHandle.advertise(s_distPub);

    SHELL_LOG("[ROS] Entering communication loop\r\n");

    xSemaphoreGive(s_rosSemaphore);

    while (true)
    {
        configASSERT(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

        s_rosNodeHandle.spinOnce();

        if (s_pendingBeaconData.size() > 0)
        {
            dwm1000::BeaconDataArray beaconDataArray;
            beaconDataArray.beacons = &s_pendingBeaconData[0];
            beaconDataArray.beacons_length = s_pendingBeaconData.size();
            s_distPub.publish(&beaconDataArray);
        }

        xSemaphoreGive(s_rosSemaphore);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void TelemSendBeaconData(const dwm1000::BeaconData& beaconData)
{
    configASSERT(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

    bool bNewBeacon = true;
    for (int i = 0; i < s_pendingBeaconData.size(); ++i)
    {
        if (s_pendingBeaconData[i].id == beaconData.id)
        {
            s_pendingBeaconData[i].dist = beaconData.dist;
            bNewBeacon = false;
            break;
        }
    }

    if (bNewBeacon)
    {
        s_pendingBeaconData.push_back(beaconData);
    }

    xSemaphoreGive(s_rosSemaphore);
}
