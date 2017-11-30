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

#define TRIGGER_PIN         12
#define ECHO_PIN            13
#define IR_PIN              26   
#define MOTOR_LEFT_A        11
#define MOTOR_LEFT_B        31
#define MOTOR_LEFT_PWM      14
#define MOTOR_RIGHT_A       32
#define MOTOR_RIGHT_B       33
#define MOTOR_RIGHT_PWM     15
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

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

Kalman kalman(0, 1023, 2, 0.125);
Kalman ir_kalman(0, 1023, 2, 0.05);

ros::NodeHandle  nh;

nav_msgs::Odometry odometry_msg;
ros::Publisher pub_odometry("/Odometry", &odometry_msg);

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

sensor_msgs::Range range_msg_K;
ros::Publisher pub_range_K( "/Kalman_ultrasound", &range_msg_K);

sensor_msgs::Range ir_msg;
ros::Publisher pub_ir( "/IR", &ir_msg);

sensor_msgs::Range ir_msg_K;
ros::Publisher pub_ir_K( "/Kalman_IR", &ir_msg_K);

char frameid[] = "/ultrasound";

char frameid_ir[] = "/ir_ranger";

long old_t;

void twistCb( const geometry_msgs::Twist &twist_msg){

  float dt = (millis() - old_t) / 1000;

  long rightTicks = encoder.getRightTicks();
  long leftTicks = encoder.getRightTicks();

  mobileCar.execute(twist_msg.linear.x, leftTicks, rightTicks, twist_msg.angular.z, dt, &leftPID, &rightPID);

  motor.drive(mobileCar.getRightPWM(), mobileCar.getLeftPWM());

  // Pusblish Odometry
  odometry_msg.header.stamp = nh.now();
  odometry_msg.pose.pose.position.x = mobileCar.getX();
  odometry_msg.pose.pose.position.y = mobileCar.getY();
  // TODO : imu quarternion
  odometry_msg.twist.twist.linear.x = mobileCar.getVr() - mobileCar.getVl();
  odometry_msg.twist.twist.angular.z = mobileCar.getTheta();
  // need convariance ??
  pub_odometry.publish(&odometry_msg);

  encoder.clearTicks();
  old_t = millis();
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/Twist", &twistCb );

void setup()
{
  old_t = millis();
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  
  nh.subscribe(sub);
  
  nh.advertise(pub_range);
  nh.advertise(pub_range_K);

  nh.advertise(pub_ir);
  nh.advertise(pub_ir_K);

  ultrasonic.setupUltrasonic();

  pinMode(IR_PIN, INPUT);
  motor.setup();
  encoder.setup();
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.261799;  //15/57.2958 radian
  range_msg.min_range = 0.02;
  range_msg.max_range = 2.0;
  
  range_msg_K.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg_K.field_of_view = 0.261799; //  15/57.2958 radian
  range_msg_K.min_range = 0.02;
  range_msg_K.max_range = 2.0;
  range_msg_K.header.frame_id = "/Kalman_ultrasound";

  ir_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_msg.header.frame_id =  frameid_ir;
  ir_msg.field_of_view = 0.01;
  ir_msg.min_range = 0.02;
  ir_msg.max_range = 1.5;

  ir_msg_K.radiation_type = sensor_msgs::Range::INFRARED;
  ir_msg_K.header.frame_id =  "/Kalman_IR";
  ir_msg_K.field_of_view = 0.01;
  ir_msg_K.min_range = 0.02;
  ir_msg_K.max_range = 1.5;

  odometry_msg.header.frame_id = "/base_link";
  odometry_msg.child_frame_id = "/base_link";
}


long range_time;

void loop()
{
  
  //publish the adc value every 50 milliseconds
  //since it takes that long for the sensor to stablize
  if ( millis() >= range_time ){
    range_msg.range = ultrasonic.updateUltrasonic() / 100.0;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);

    range_msg_K.range = kalman.execute(range_msg.range);
    range_msg_K.header.stamp = nh.now();
    pub_range_K.publish(&range_msg_K);

    int val = analogRead(IR_PIN); // read the sensor
    float data = (10650.08 * pow(val, -0.935) - 10) / 100.0;
    ir_msg.range = data;
    ir_msg.header.stamp = nh.now();
    pub_ir.publish(&ir_msg);

    ir_msg_K.range = ir_kalman.execute(data);
    ir_msg_K.header.stamp = nh.now();
    pub_ir_K.publish(&ir_msg_K);
    
    
    range_time =  millis() + 60;
  }
  
  nh.spinOnce();
}
