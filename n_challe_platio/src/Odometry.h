#pragma once
#include"Vector.h"

class Odmetry{
public:
  Odmetry(const double wheel_width=555.0,
          const double encoder_ppr=8192,
          const double wheel_size=315.0);
  Vector update(long l_enc,long r_enc,double gyro_yaw);
  Vector vec;
private:
  double pre_rad;
  long pre_pul;
};

Odmetry::Odmetry(const double wheel_width,
                 const double encoder_ppr,
                 const double wheel_size){
  this->wheel_width=wheel_width;
  this->encoder_ppr=encoder_ppr;
  this->wheel_size=wheel_size;
}

Vector Odmetry::update(long l_enc,long r_enc,double gyro_yaw){
  //odometry
  long rpul= -l_enc;
  long lpul= r_enc;

  double angle_rad=-gyro_yaw;

  long now_pul=rpul+lpul;
  double diff_pos=((now_pul-pre_pul)/2.0)*wheel_size*PI/encoder_ppr/1000.0;
  pre_pul=now_pul;

  vec.y+=diff_pos*cos((angle_rad+pre_rad)/2.0-angle_offset);
  vec.x+=diff_pos*sin((angle_rad+pre_rad)/2.0-angle_offset);
  vec.yaw=angle_rad;
  pre_rad=angle_rad;
  return vec;
}
