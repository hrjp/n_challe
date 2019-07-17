#include "Cytron_MD13S.h"
#include"ps3i2clib.h"
//#include<cout.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#include "Encoders.h"
#include<cout.h>
ENCODERS Encoders(45,48);

int x, y, z;
MD13S lmo(6, 5); //(PWM_PIN,invert_PIN)
MD13S rmo(8,7);
PS3I2C ps(0x73);
int lp=1500,rp=1500;
double pos_x,pos_y,angle_offset,angle_deg,angle_rad;

void messageCb(const geometry_msgs::Twist& twist) {
  const float linear_x = twist.linear.x;
  const float angle_z = 0.7*twist.angular.z;
  rmo.writeMicroseconds(1500+150*(linear_x+angle_z));
  lmo.writeMicroseconds(1500-150*(linear_x-angle_z));
  if(twist.angular.x){
    pos_x=0;
    pos_y=0;
    angle_offset=angle_rad;
  }
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);

geometry_msgs::TransformStamped t;
geometry_msgs::Twist send_pos;
//ros::Publisher chatter("robot_msg", &send_pos);

tf::TransformBroadcaster broadcaster;

 
char base_link[] = "/base_link";
char odom[] = "/odom";


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
  analogWriteFrequency(6, 20000);
  analogWriteFrequency(8, 20000);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  nh.initNode();
  nh.subscribe(sub);
  broadcaster.init(nh);

  Encoders.Encoder1.set(4096);
  Encoders.Encoder2.set(4096);
  Encoders.set(8192);
  
  // ps.set();
  // put your setup code here, to run once:
  lmo.set();
  rmo.set();
}

void loop() {

  //odometry
  long rpul= Encoders.Encoder1.read_pulse();
  long lpul= -Encoders.Encoder2.read_pulse();


  const double wheel_width=480.0;
  const double encoder_ppr=4096;
  const double wheel_size=150.0;
  static unsigned long pre_t=0;
  double dt=(micros()-pre_t)/1000000.0;
  pre_t=micros();
  static double pre_rad;
  angle_deg=((rpul-lpul)/2.0)*180.0*wheel_size/(encoder_ppr*wheel_width*0.5);
  angle_rad=angle_deg*PI/180;
  static long pre_pul;
  long now_pul=rpul+lpul;
  double diff_pos=((now_pul-pre_pul)/2.0)*wheel_size*PI/encoder_ppr/1000.0;
  pre_pul=now_pul;

  pos_y+=diff_pos*cos(angle_rad);
  pos_x+=diff_pos*sin(angle_rad);
  double vel_x=diff_pos*cos(angle_rad)/dt;
  double vel_y=diff_pos*sin(angle_rad)/dt;
  double vel_z=(angle_rad-pre_rad)/dt;
  pre_rad=angle_rad;
//tf

  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  //t.transform.translation.x = -pos_x; 
  //t.transform.translation.y = pos_y;
  t.transform.translation.x = pos_y; 
  t.transform.translation.y = pos_x;
  double q[5];
  EulerAnglesToQuaternion(0,0,angle_rad-angle_offset,q[0],q[1],q[2],q[3]);
  t.transform.rotation.x = q[1];
  t.transform.rotation.y = q[2]; 
  t.transform.rotation.z = q[3]; 
  t.transform.rotation.w = q[0];  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);

  //twist_pub

  send_pos.linear.x=-pos_x;
  send_pos.linear.y=pos_y;
  send_pos.linear.z=angle_rad-angle_offset;
  send_pos.angular.x=-vel_x;
  send_pos.angular.y=vel_y;
  send_pos.angular.z=vel_z;
  //chatter.publish( &send_pos );
  
   nh.spinOnce();
}

//platformio_add