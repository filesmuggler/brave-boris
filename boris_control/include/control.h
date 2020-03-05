#ifndef control_h
#define control_h

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

using namespace std;
using namespace ros;

std_msgs::Int16 throttleMsg;
std_msgs::Int16 rotationMsg;

Publisher throttlePub, rotationPub;

void SigintHandler(int sig);
void vehicleStartup();
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

#endif
