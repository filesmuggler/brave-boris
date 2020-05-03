#include "MPU9250.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

//ROS
ros::NodeHandle nh;
sensor_msgs::Imu IMUMsg;
sensor_msgs::MagneticField MagMsg;
sensor_msgs::Temperature TempMsg;
ros::Publisher IMUPub("/imu/data_raw", &IMUMsg);
ros::Publisher MagPub("/imu/mag", &MagMsg);
ros::Publisher TempPub("temperature", &TempMsg);

//IMU
MPU9250 IMU(Wire,0x68);
int status;

//LED blink
const int ledPin =  13;
int ledState = LOW;
long previousMillis = 0;
long interval = 1000;

void setup() 
{
  //ROS
  nh.initNode();
  nh.advertise(IMUPub);
  nh.advertise(MagPub);
  nh.advertise(TempPub);

  //IMU
  status = IMU.begin();
  if (status < 0) 
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
  }

  //LED blink
  pinMode(ledPin, OUTPUT);
}

void loop() 
{
  //IMU
  IMU.readSensor();

  //ROS
  IMUMsg.header.stamp = nh.now();
  IMUMsg.header.frame_id = "base_link";
  MagMsg.header.stamp = nh.now();
  MagMsg.header.frame_id = "base_link";

  //covariances

  //IMU message
  //orientation
  IMUMsg.orientation_covariance[0] = 0.0025;
  IMUMsg.orientation_covariance[4] = 0.0025;
  IMUMsg.orientation_covariance[8] = 0.0025;
  //velocity
  IMUMsg.angular_velocity_covariance[0] = 0.0025;
  IMUMsg.angular_velocity_covariance[4] = 0.0025;
  IMUMsg.angular_velocity_covariance[8] = 0.0025;
  //acceleration  
  IMUMsg.linear_acceleration_covariance[0] = 0.0025;
  IMUMsg.linear_acceleration_covariance[4] = 0.0025;
  IMUMsg.linear_acceleration_covariance[8] = 0.0025;

  //magnetic field message
  MagMsg.magnetic_field_covariance[0] = 0.0025;
  MagMsg.magnetic_field_covariance[4] = 0.0025;
  MagMsg.magnetic_field_covariance[8] = 0.0025;
  
  //values

  //IMU message
  //accelerometer
  IMUMsg.linear_acceleration.x = IMU.getAccelX_mss();
  IMUMsg.linear_acceleration.y = IMU.getAccelY_mss();
  IMUMsg.linear_acceleration.z = IMU.getAccelZ_mss();
  //gyroscope
  IMUMsg.angular_velocity.x = IMU.getGyroX_rads();
  IMUMsg.angular_velocity.y = IMU.getGyroY_rads();
  IMUMsg.angular_velocity.z = IMU.getGyroZ_rads();
  //magnetometer
  IMUMsg.orientation.x = IMU.getMagX_uT();
  IMUMsg.orientation.y = IMU.getMagY_uT();
  IMUMsg.orientation.z = IMU.getMagZ_uT();

  IMUPub.publish(&IMUMsg);

  //magnetic field message
  MagMsg.magnetic_field.x = IMU.getMagX_uT();
  MagMsg.magnetic_field.y = IMU.getMagY_uT();
  MagMsg.magnetic_field.z = IMU.getMagZ_uT();
  
  MagPub.publish(&MagMsg);

  //temperature
  
  TempMsg.variance = 0;
  TempMsg.header.stamp = nh.now();
  TempMsg.header.frame_id = "base_link";
  TempMsg.temperature = IMU.getTemperature_C();

  TempPub.publish(&TempMsg);

  //LED blink
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;
    if (ledState == LOW)
      ledState = HIGH;
    else
      ledState = LOW;
    digitalWrite(ledPin, ledState);
  }

  delay(100);
  nh.spinOnce();
}
