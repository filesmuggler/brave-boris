#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

ros::NodeHandle nh;
std_msgs::Int16 forward, angle;

Servo servo1, servo2;

void throttleCb(const std_msgs::Int16& throttle){
  forward.data = throttle.data;
}

void rotationCb(const std_msgs::Int16& rotation){
  angle.data = rotation.data;
}

ros::Subscriber<std_msgs::Int16> throttleSub("throttle", &throttleCb);
ros::Subscriber<std_msgs::Int16> rotationSub("rotation", &rotationCb);

void setup() {
  nh.initNode();
  nh.subscribe(throttleSub);
  nh.subscribe(rotationSub);

  servo1.attach(6);
  servo2.attach(9);

  forward.data = 128;
  angle.data = 128;
}

void loop() {
  servo1.write(forward.data);
  servo2.write(angle.data);

  delay(30);
  nh.spinOnce();
}
