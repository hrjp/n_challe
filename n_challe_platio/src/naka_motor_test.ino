//#include <Arduino.h>
#include "Cytron_MD13S.h"
#include"ps3i2clib.h"
//#include<cout.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>


#include "Gyro_fast.h"
#include "Encoders.h"
#include<cout.h>
#include"Vector.h"
#include"PID_lib.h"
#include "Pixy_analog.h"


//1:rosからのcmd_velで動く
//0:rosを介さずコントローラの値で動く




ENCODERS Encoders(45,48);
Gyro_fast gyro;

int x, y, z;
MD13S lmo(6, 5); //(PWM_PIN,invert_PIN)
MD13S rmo(8,7);
PS3I2C ps(0x73);
int lp=1500,rp=1500;
double pos_x,pos_y,angle_offset,angle_deg,angle_rad;

Vector body_vel;
Vector target_vel;

//* 190920_1730
// PID r_vel(100.0,1500,350);
// PID l_vel(100.0,1500,350);

//* 190920_1731, GOOD?
//PID r_vel(100.0,1000,350);
//PID l_vel(120.0,1000,350);

//* 190920_1731, Bad, Iの値で悪化しているよう
// PID r_vel(120.0,1200,350);
// PID l_vel(120.0,1200,350);
PID r_vel(110.0,640,350);
PID l_vel(110.0,640,350);

PID pixypid(0.3,0.0,0.1);
PID dispid(100.0,0.0,10);

bool using_cmd_vel;
//int ditect_mode;
//double human_dis;
//Pixy_analog pixy(A7);

void messageCb(const geometry_msgs::Twist& twist) {
  //const float linear_x = 6*twist.linear.x;
  //const float angle_z = 0.5*twist.angular.z;
  //if(using_cmd_vel){
    target_vel.y=2.0*twist.linear.x;
    target_vel.yaw=0.6*twist.angular.z;
    target_vel.x=twist.linear.y;
    //human_dis=twist.linear.y;
    //ditect_mode=twist.angular.x;
  //}
  //rmo.writeMicroseconds(1500+100*(linear_x+angle_z));
  //lmo.writeMicroseconds(1500-100*(linear_x-angle_z));
  /*
  if(twist.angular.x){
    pos_x=0;
    pos_y=0;
    angle_offset=angle_rad;
  }
  */
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("final_cmd_vel", &messageCb);

geometry_msgs::TransformStamped t;
geometry_msgs::Twist send_pos;
//ros::Publisher chatter("robot_msg", &send_pos);

tf::TransformBroadcaster broadcaster;


char base_link[] = "/base_link";
char odom[] = "/odom";

std_msgs::Float32 odometry;
ros::Publisher odometry_pub("robot_odom", &odometry);


void EulerAnglesToQuaternion(double roll, double pitch, double yaw,double& q0, double& q1, double& q2, double& q3){
    double cosRoll = cos(roll / 2.0);
    double sinRoll = sin(roll / 2.0);
    double cosPitch = cos(pitch / 2.0);
    double sinPitch = sin(pitch / 2.0);
    double cosYaw = cos(yaw / 2.0);
    double sinYaw = sin(yaw / 2.0);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

void setup() {
   Wire.begin();
   Wire.setSDA(34);
   Wire.setSCL(33);
   Wire.setClock(400000UL);
   gyro.set();
  analogWriteFrequency(6, 20000);
  analogWriteFrequency(8, 20000);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  nh.initNode();
  nh.subscribe(sub);
  broadcaster.init(nh);
  pinMode(30,INPUT_PULLUP);
  Encoders.Encoder1.set(8192);
  Encoders.Encoder2.set(8192);
  Encoders.set(8192);
  Serial.begin(115200);
  nh.advertise(odometry_pub);

  //ps.set();
  // put your setup code here, to run once:
  lmo.set();
  rmo.set();
  r_vel.max_i(0.3);
  l_vel.max_i(0.3);
}

void loop() {
  gyro.update();
  ps.update();
  //pixy.update();
  using_cmd_vel=digitalRead(30);
/*
  if(!using_cmd_vel){
    target_vel.y=0.8*((255-ps.A_Ly())/127.5-1.0);
    target_vel.yaw=0.7*((255-ps.A_Rx())/127.5-1.0);
  }
*/


  //odometry
  long rpul= -Encoders.Encoder1.read_pulse();
  long lpul= Encoders.Encoder2.read_pulse();
  //旧機体
  /*
  const double wheel_width=480.0;
  const double encoder_ppr=4096;
  const double wheel_size=150.0;
  */
  //新機体
  const double wheel_width=555.0;
  const double encoder_ppr=8192;
  const double wheel_size=315.0;


  static unsigned long pre_t=0;
  double dt=(micros()-pre_t)/1000000.0;
  pre_t=micros();
  static double pre_rad;
  angle_deg=((rpul-lpul)/2.0)*180.0*wheel_size/(encoder_ppr*wheel_width*0.5);
  //angle_rad=angle_deg*PI/180;
  angle_rad=-gyro.rad();
  static long pre_pul;
  long now_pul=rpul+lpul;
  double diff_pos=((now_pul-pre_pul)/2.0)*wheel_size*PI/encoder_ppr/1000.0;
  pre_pul=now_pul;

  pos_y+=diff_pos*cos((angle_rad+pre_rad)/2.0-angle_offset);
  pos_x+=diff_pos*sin((angle_rad+pre_rad)/2.0-angle_offset);

  double vel_liner=diff_pos/dt;
  double vel_x=diff_pos*cos(angle_rad)/dt;
  double vel_y=diff_pos*sin(angle_rad)/dt;
  double vel_z=(angle_rad-pre_rad)/dt;
  pre_rad=angle_rad;
  body_vel.y=vel_liner;
  body_vel.yaw=vel_z;

//タイヤ回転速度の計算
/*
static long pre_rpul,pre_lpul;
double r_rot=(rpul-pre_rpul)*wheel_size*PI/encoder_ppr/1000.0/dt;
double l_rot=(lpul-pre_lpul)*wheel_size*PI/encoder_ppr/1000.0/dt;
pre_rpul=rpul;
pre_lpul=lpul;
*/
double r_rot=-Encoders.Encoder1.read_rpm()*PI/60.0*wheel_size/1000.0;
double l_rot=Encoders.Encoder2.read_rpm()*PI/60.0*wheel_size/1000.0;


//tf

  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  //t.transform.translation.x = -pos_x;
  //t.transform.translation.y = pos_y;
  t.transform.translation.x = pos_y;
  t.transform.translation.y = pos_x;
  //t.transform.translation.x = -pos_x;
  //t.transform.translation.y = -pos_y;
  double qu[5];
  EulerAnglesToQuaternion(0,0,angle_rad-angle_offset,qu[0],qu[1],qu[2],qu[3]);

  t.transform.rotation.x = qu[1];
  t.transform.rotation.y = qu[2];
  t.transform.rotation.z = qu[3];
  t.transform.rotation.w = qu[0];
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);


  //速度制御
  const int max_power=300;
  r_vel.update(r_rot,target_vel.y+0.5*wheel_width/1000.0*target_vel.yaw);
  l_vel.update(l_rot,(target_vel.y-0.5*wheel_width/1000.0*target_vel.yaw));
  int dir_r=(target_vel.y+target_vel.yaw)>0;
  int dir_l=(target_vel.y-target_vel.yaw)>0;

  if(abs(r_rot)<0.01&&abs(l_rot)<0.01){
    r_vel.max_i(0.1);
    l_vel.max_i(0.1);
  }
  else{
    r_vel.max_i(0.3);
    l_vel.max_i(0.3);
  }
  /*
  //人検出の処理
  const int max_power_dis=110;
  pixypid.update(pixy.point_x(),0);
  dispid.update(target_vel.x,0.5);

  int dis_result=constrain(dispid.result_val(),-max_power_dis,max_power_dis);

  //人検出シーケンス
  static int sik=0;
  static unsigned long wait_t=0;
  static int pre_d=0;
  static int gomi_mode=0;
  const long ditect_time=5;
  static int d_time=0;
  if(sik==0&&(ditect_mode-pre_d==1)){
    wait_t=millis();
    sik++;
  }
  if(sik==1){
    if(wait_t+20000<millis()){
      sik++;
    }
  }
  if(sik==2){
    gomi_mode=true;
    sik++;
  }
  if(sik==3){
    if(abs(target_vel.x)<0.8){
      if(pixy.finish(10)){
        d_time++;
      }
    }
    else{
      d_time=0;
    }
    if(d_time>ditect_time){
      gomi_mode=2;
    }
  }
  if(!ditect_mode){
    sik=0;
    gomi_mode=false;
  }



  pre_d=ditect_mode;
*/
//モータへの出力/////////////////////////////////////
  if(using_cmd_vel){
//人検出
/*
    if(gomi_mode==1){
      rmo.writeMicroseconds(1500+constrain(pixypid.result_val()-dis_result,-max_power,max_power));
      lmo.writeMicroseconds(1500+constrain(pixypid.result_val()+dis_result,-max_power,max_power));
    }
    else if(gomi_mode==2){
      rmo.writeMicroseconds(1500);
      lmo.writeMicroseconds(1500);
    }*/
//cmd_velで走行
    //else{
      if(-0.03<(target_vel.y+0.5*wheel_width/1000.0*target_vel.yaw)&&(target_vel.y+0.5*wheel_width/1000.0*target_vel.yaw)<0.03){
        rmo.writeMicroseconds(1500);
        r_vel.reset_i();
      }
      else{
        rmo.writeMicroseconds(1500+constrain(r_vel.result_val(),-max_power*!dir_r,max_power*dir_r));
      }
      if(-0.03<(target_vel.y-0.5*wheel_width/1000.0*target_vel.yaw)&&(target_vel.y-0.5*wheel_width/1000.0*target_vel.yaw)<0.03){
        lmo.writeMicroseconds(1500);
        l_vel.reset_i();
      }
      else{
        lmo.writeMicroseconds(1500-constrain(l_vel.result_val(),-max_power*!dir_l,max_power*dir_l));
      }
    }
  //}
//コントローラから操作
  else{
    rmo.writeMicroseconds(map(0.6*(ps.A_Ly()+ps.A_Rx()-255),127,-127,1500-max_power,1500+max_power));
    lmo.writeMicroseconds(map(0.6*(ps.A_Ly()-ps.A_Rx()),-127,127,1500-max_power,1500+max_power));
  }
  odometry.data = 1.0;
  odometry_pub.publish(&odometry);
   nh.spinOnce();
   //cout<<r_rot<<","<<target_vel.y+0.5*wheel_width/1000.0*target_vel.yaw<<","<<gyro.rad()<<endl;
   //cout<<"X="<<pos_x<<"Y="<<pos_y<<"YAW="<<angle_rad-angle_offset<<endl;
/*
   if(ps.C_Select()){
    pos_x=0;
    pos_y=0;
    angle_offset=angle_rad;
  }*/

}

//platformio_add
