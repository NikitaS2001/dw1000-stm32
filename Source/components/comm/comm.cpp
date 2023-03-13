#include "comm.h"

#include "dwm1000_msgs/BeaconDataArray.h"
#include "ros.h"

#include <FreeRTOS.h>
#include <assert.h>
#include <semphr.h>
#include <task.h>

#include <stdio.h>

static xSemaphoreHandle s_rosSemaphore = NULL;

static ros::NodeHandle s_rosNodeHandle;
static dwm1000_msgs::BeaconDataArray s_beaconDataMsg;
static ros::Publisher s_distPub("dwm1000/beacon_data", &s_beaconDataMsg);

void CommTask(void* pvParameters)
{
    printf("[ROS] Initializing...\r\n");

    s_rosSemaphore = xSemaphoreCreateMutex();

    assert(s_rosSemaphore != NULL);
    assert(xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) == pdTRUE);

    s_rosNodeHandle.initNode();
    s_rosNodeHandle.advertise(s_distPub);

    printf("[ROS] Entering communication loop\r\n");

    xSemaphoreGive(s_rosSemaphore);

    while (true)
    {
        if (xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) != pdTRUE)
        {
            printf("[ROS] Could not take semaphore in comm loop!!\r\n");
            continue;
        }

        // str_msg.data = hello;
        // chatter.publish(&str_msg);

        s_rosNodeHandle.spinOnce();

        xSemaphoreGive(s_rosSemaphore);

        vTaskDelay(1000);
    }

    vTaskDelete(NULL);
}

void CommSendBeaconDataArray(void* dataArray)
{
    if (xSemaphoreTake(s_rosSemaphore, portMAX_DELAY) != pdTRUE)
    {
        printf("[ROS] Could not take semaphore while sending beacon data!!\r\n");
        return;
    }

    s_distPub.publish((dwm1000_msgs::BeaconDataArray*)dataArray);

    xSemaphoreGive(s_rosSemaphore);
}
