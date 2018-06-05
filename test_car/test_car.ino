#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
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
#define MOTOR_LEFT_A        32
#define MOTOR_LEFT_B        33
#define MOTOR_LEFT_PWM      PM_5
#define MOTOR_RIGHT_A       11
#define MOTOR_RIGHT_B       31
#define MOTOR_RIGHT_PWM     PM_4
#define ENCODER_LEFT_A      71
#define ENCODER_LEFT_B      72
#define ENCODER_RIGHT_A     73
#define ENCODER_RIGHT_B     74

Motordriver motor(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_PWM, MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_PWM);

Encoder encoder(ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_A, ENCODER_RIGHT_B);

MobileCar mobileCar(0.0, 0.0, 0.0);     // x, y, theta

/****PID for Kp Kd Ki ,  need test ****/

PID leftPID(4.0, 0.01, 0.1);
PID rightPID(4.0, 0.01, 0.1);

/**************************************/


//ros::NodeHandle_<TivaCHardware,5,5,2000,2000>  nh;

ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

nav_msgs::Odometry odometry_msg;
ros::Publisher pub_odometry("/odom", &odometry_msg);


std_msgs::Float64 pwmR_msg;
ros::Publisher pub_pwmR("/rightTicks", &pwmR_msg);

std_msgs::Float64 pwmL_msg;
ros::Publisher pub_pwmL("/leftTicks", &pwmL_msg);

std_msgs::Float64 theta_msg;
ros::Publisher pub_theta("/theta", &theta_msg);

unsigned long old_t;

volatile long prev_leftTicks = 0;
volatile long prev_rightTicks = 0;

double v = 0.0;
double w = 0.0;

void publishOdometry(){
  
  long rightTicks = encoder.getRightTicks();
  long leftTicks = encoder.getLeftTicks();
  encoder.clearTicks();
  
  double dt = (millis() - old_t) / 1000.0;
  old_t = millis();
  
  mobileCar.execute(v, w, dt, &leftPID, &rightPID, leftTicks, rightTicks);

  motor.drive(mobileCar.getRightPWM(), mobileCar.getLeftPWM());
 
  double diff = mobileCar.getX();
  mobileCar.updateOdometry(leftTicks, rightTicks, dt);
  diff = mobileCar.getX() - diff;
  
  
  // Pusblish Odometry
  odometry_msg.header.stamp = nh.now();
  odometry_msg.pose.pose.position.x = mobileCar.getX();
  odometry_msg.pose.pose.position.y = mobileCar.getY();
  odometry_msg.pose.pose.orientation = tf::createQuaternionFromYaw(mobileCar.getTheta());
  //odometry_msg.pose.pose.orientation.z = sin((mobileCar.getTheta() / 2.0) * DEG_TO_RAD);
  //odometry_msg.pose.pose.orientation.w = cos((mobileCar.getTheta() / 2.0) * DEG_TO_RAD);
  // TODO : imu quarternion
  odometry_msg.twist.twist.linear.x = mobileCar.getVx(); //mobileCar.getRadius() * (mobileCar.getVr() + mobileCar.getVl()) * cos(mobileCar.getTheta()) / 2.0;
  odometry_msg.twist.twist.linear.y = mobileCar.getVy(); //mobileCar.getRadius() * (mobileCar.getVr() + mobileCar.getVl()) * sin(mobileCar.getTheta()) / 2.0;
  odometry_msg.twist.twist.angular.z = mobileCar.getW(); //mobileCar.getRadius() * (mobileCar.getVr() - mobileCar.getVl()) / mobileCar.getL();
  // need convariance ??
  pub_odometry.publish(&odometry_msg);


  //tf
  t.header.frame_id = "/odom";
  t.child_frame_id = "/base_link";
  t.transform.translation.x = odometry_msg.pose.pose.position.x; 
  t.transform.translation.y = odometry_msg.pose.pose.position.y; 
  t.transform.rotation = odometry_msg.pose.pose.orientation;
  //t.transform.rotation.z = odometry_msg.pose.pose.orientation.z; 
  //t.transform.rotation.w = odometry_msg.pose.pose.orientation.w;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);

  pwmR_msg.data = rightTicks;
  pub_pwmR.publish(&pwmR_msg);
  pwmL_msg.data = leftTicks;
  pub_pwmL.publish(&pwmL_msg);
  
  theta_msg.data = mobileCar.getObjRightTicks();
  pub_theta.publish(&theta_msg);

  prev_leftTicks = leftTicks;
  prev_rightTicks = rightTicks;

}

void twistCb( const geometry_msgs::Twist &twist_msg){

  float dt = 0.02; // unused

  v = sqrt(pow(twist_msg.linear.x, 2) + pow(twist_msg.linear.y, 2));

  v = twist_msg.linear.x > 0? v:-v;

  w = twist_msg.angular.z;
  
  

}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twistCb );

void setup()
{
  old_t = millis();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.subscribe(sub);

  nh.advertise(pub_odometry);
  nh.advertise(pub_pwmR);
  nh.advertise(pub_pwmL);
  nh.advertise(pub_theta);

  //nh_.getHardware()->setBaud(115200);
  //nh_.initNode();
  broadcaster.init(nh);

  pinMode(IR_PIN, INPUT);
  motor.setup();
  encoder.setup();

  
  odometry_msg.header.frame_id = "/odom";
  odometry_msg.child_frame_id = "/base_link";
}


long range_time;

void loop()
{

  publishOdometry();

  nh.spinOnce();
  //nh_.spinOnce();

  //delay(925);
}
