#include<Arduino.h>
#include <ros.h>
#include"GPS.h"
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;
std_msgs::Int32MultiArray int_pub_array;
ros::Publisher int_pub("int_sensor_data", &int_pub_array);
GPS gps;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
   //Wire.setSDA(34);
   //Wire.setSCL(33);
  gps.set();
  int_pub_array.data = (int32_t *)malloc(sizeof(int32_t)*3);
  int_pub_array.data_length=3;
  int_pub_array.data[0]=0.0;
  int_pub_array.data[1]=0.0;
  int_pub_array.data[2]=0.0;
  nh.advertise(int_pub);
}

void loop() {
  // put your main code here, to run repeatedly:
  gps.update();

  //GPS
  int_pub_array.data[0]=gps.latitude;
  int_pub_array.data[1]=gps.longitude;
  int_pub_array.data[2]=gps.altitude;

  //データのpublish
  int_pub.publish(&int_pub_array);
  nh.spinOnce();
}