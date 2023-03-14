#include "comm.h"

#include "shell/shell.h"

#include "ros_lib/dwm1000/BeaconDataArray.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <semphr.h>
#include <task.h>

#include <stdio.h>

xSemaphoreHandle s_rosSemaphore = NULL;

ros::NodeHandle s_rosNodeHandle;
dwm1000::BeaconDataArray s_beaconDataMsg;
ros::Publisher s_distPub("dwm1000/beacon_data", &s_beaconDataMsg);

std::vector<dwm1000::BeaconData> s_pendingBeaconData;

void CommTask(void* pvParameters)
{
    SHELL_LOG("[ROS] Initializing...\r\n");

    s_rosSemaphore = xSemaphoreCreateMutex();

    assert(s_rosSemaphore != NULL);
    assert(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

    s_rosNodeHandle.initNode();
    s_rosNodeHandle.advertise(s_distPub);

    SHELL_LOG("[ROS] Entering communication loop\r\n");

    xSemaphoreGive(s_rosSemaphore);

    while (true)
    {
        assert(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

        s_rosNodeHandle.spinOnce();

        /*if (s_pendingBeaconData.size() > 0)
        {
            dwm1000::BeaconDataArray beaconDataArray;
            beaconDataArray.beacons = &s_pendingBeaconData[0];
            beaconDataArray.beacons_length = s_pendingBeaconData.size();
            s_distPub.publish(&beaconDataArray);

            s_pendingBeaconData.clear();
        }*/

        xSemaphoreGive(s_rosSemaphore);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void CommSendBeaconDataArray(const std::vector<dwm1000::BeaconData>& beaconDataArray)
{
    //assert(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

    //s_pendingBeaconData = beaconDataArray;

    //xSemaphoreGive(s_rosSemaphore);
}
