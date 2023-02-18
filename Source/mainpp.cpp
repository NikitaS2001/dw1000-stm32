#include "mainpp.h"

#include "deca_lib/deca_sleep.h"

#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

#include <stdio.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop(void)
{
  printf("Publish\r\n");
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();

  deca_sleep(1000);
}
