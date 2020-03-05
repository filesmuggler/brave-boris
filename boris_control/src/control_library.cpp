#include "../include/control.h"

void SigintHandler(int sig)
{
  ROS_INFO_STREAM("Control Node Is Shutting Down");
  // Stopping everything
  throttleMsg.data = 127;
  rotationMsg.data = 0;
  throttlePub.publish(throttleMsg);
  rotationPub.publish(rotationMsg);
  shutdown();
}

void vehicleStartup()
{
  ROS_INFO_STREAM("Initiating startup sequence");
  throttleMsg.data = 127;
  rotationMsg.data = 0;
  ROS_INFO_STREAM("Vehicle reached set speed");
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // 0 - lewa galka, lewo/prawo
  // 1 - lewa galka, gora/dol
  // 3 - prawa galka, lewo/prawo
  // 4 - prawa galka, gora/dol

  // -1.0:1.0 to 0:200
  throttleMsg.data = (float)((joy->axes[1] + 1) * 0.5) * 200;

  // -1.0:1.0 to -50:50
  rotationMsg.data = (float)(joy->axes[0] * 50);
}
