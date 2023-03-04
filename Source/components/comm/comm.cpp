#include "comm.h"

#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

#include <FreeRTOS.h>
#include <task.h>

#include <stdio.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

void CommInit(void)
{
    nh.initNode();
    nh.advertise(chatter);
}

void loop(void)
{
    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();

    vTaskDelay(1000);
}
