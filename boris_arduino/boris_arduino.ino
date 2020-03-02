// PWM frequency: 490 Hz (pins 5 and 6: 980 Hz)

// 0% - max backward
// 50% - neutral
// 100% - max forward

#include <ros.h>
#include <std_msgs/Int16.h>

#define LEFT_PIN 10
#define RIGHT_PIN 11

ros::NodeHandle nh;

std_msgs::Int16 left, right;

void throttleCb(const std_msgs::Int16& throttle)
{
  left.data = throttle.data;
  right.data = throttle.data;
}

void rotationCb(const std_msgs::Int16& rotation)
{
  left.data = - rotation.data;
  right.data = + rotation.data;
}

ros::Subscriber<std_msgs::Int16> throttleSub("throttle", &throttleCb);
ros::Subscriber<std_msgs::Int16> rotationSub("rotation", &rotationCb);

void setup() 
{
  nh.initNode();
  nh.subscribe(throttleSub);
  nh.subscribe(rotationSub);

  pinMode(LEFT_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);

  //neutral values (2.5V, 50%)
  left.data = 128;
  right.data = 128;
}

void loop() 
{
  analogWrite(LEFT_PIN, left.data);
  analogWrite(RIGHT_PIN, right.data);

  delay(30);
  
  nh.spinOnce();
}
