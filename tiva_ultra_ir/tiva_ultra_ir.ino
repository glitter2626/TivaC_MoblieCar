#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <Ultrasonic.h>
#include <Kalman.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     13
#define IR_PIN       26   //A0

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);

Kalman kalman(0, 1023, 2, 0.125);
Kalman ir_kalman(0, 1023, 2, 0.05);

ros::NodeHandle  nh;

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

void setup()
{
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_range_K);

  nh.advertise(pub_ir);
  nh.advertise(pub_ir_K);

  ultrasonic.setupUltrasonic();

  //pinMode(IR_PIN, INPUT);
  
  
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
