#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <Ultrasonic.h>
#include <Kalman.h>
#include <Encoder.h>
#include <Motordriver.h>
#include <MobileCar.h>
#include <PID.h>
#include <std_msgs/Float64.h>

#define TRIGGER_PIN         12
#define ECHO_PIN            13
#define IR_PIN              26   
#define MOTOR_LEFT_A        11
#define MOTOR_LEFT_B        31
#define MOTOR_LEFT_PWM      PM_4
#define MOTOR_RIGHT_A       32
#define MOTOR_RIGHT_B       33
#define MOTOR_RIGHT_PWM     PM_5
#define ENCODER_LEFT_A      71
#define ENCODER_LEFT_B      72
#define ENCODER_RIGHT_A     73
#define ENCODER_RIGHT_B     74

Motordriver motor(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_PWM, MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_PWM);

Encoder encoder(ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B);

MobileCar mobileCar(0.0, 0.0, 0.0);     // x, y, theta

/****PID for Kp Kd Ki ,  need test ****/

PID leftPID(0.1, 0.01, 0.01);
PID rightPID(0.1, 0.01, 0.01);

/**************************************/


ros::NodeHandle_<TivaCHardware,5,5,2000,2000>  nh;

nav_msgs::Odometry odometry_msg;
ros::Publisher pub_odometry("/Odometry", &odometry_msg);


std_msgs::Float64 float_msg;
ros::Publisher pub_float("/ggggg", &float_msg);

unsigned long old_t;

void publishOdometry(){

  long rightTicks = encoder.getRightTicks();
  long leftTicks = encoder.getLeftTicks();
  encoder.clearTicks();

  float dt = 0.2;

  mobileCar.updateOdometry(leftTicks, rightTicks, dt);
  
  // Pusblish Odometry
  odometry_msg.header.stamp = nh.now();
  odometry_msg.pose.pose.position.x = mobileCar.getX();
  odometry_msg.pose.pose.position.y = mobileCar.getY();
  odometry_msg.pose.pose.orientation.z = sin((mobileCar.getTheta() / 2.0) * DEG_TO_RAD);
  odometry_msg.pose.pose.orientation.w = cos((mobileCar.getTheta() / 2.0) * DEG_TO_RAD);
  // TODO : imu quarternion
  odometry_msg.twist.twist.linear.x = mobileCar.getRadius() * (mobileCar.getVr() + mobileCar.getVl()) * cos(mobileCar.getTheta() * DEG_TO_RAD) / 2.0;
  odometry_msg.twist.twist.linear.y = mobileCar.getRadius() * (mobileCar.getVr() + mobileCar.getVl()) * sin(mobileCar.getTheta() * DEG_TO_RAD) / 2.0;
  odometry_msg.twist.twist.angular.z = mobileCar.getRadius() * (mobileCar.getVr() - mobileCar.getVl()) / mobileCar.getL();
  // need convariance ??
  pub_odometry.publish(&odometry_msg);

}

void twistCb( const geometry_msgs::Twist &twist_msg){

  float dt = 0.2; //(millis() - old_t) / (float)1000;

  double v = sqrt(pow(twist_msg.linear.x, 2) + pow(twist_msg.linear.y, 2));

  mobileCar.execute(v, twist_msg.angular.z, dt, &leftPID, &rightPID);

  motor.drive(mobileCar.getRightPWM(), mobileCar.getLeftPWM());
  
  float_msg.data = dt;
  pub_float.publish(&float_msg);
  
  old_t = millis();
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/Twist", &twistCb );

void setup()
{
  old_t = millis();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.subscribe(sub);

  nh.advertise(pub_odometry);
  nh.advertise(pub_float);

  pinMode(IR_PIN, INPUT);
  motor.setup();
  encoder.setup();

  
  odometry_msg.header.frame_id = "/base_link";
  odometry_msg.child_frame_id = "/base_link";
}


long range_time;

void loop()
{
  if(millis() >= range_time){

    publishOdometry();

    range_time = millis() + 200;
  }
  
  nh.spinOnce();
}
