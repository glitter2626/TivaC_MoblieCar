#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


ros::NodeHandle  nh;

sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu( "/imu", &imu_msg);

char frameid[] = "/imu";


//Creating MPU6050 Object
MPU6050 accelgyro(0x68);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DMP options
//Set true if DMP init was successful
bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP packet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];

int16_t ax, ay, az;
int16_t gx, gy, gz;

Quaternion q;


//INTERRUPT DETECTION ROUTINE

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}


void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_imu);

  imu_msg.header.frame_id =  frameid;
 
  Setup_MPU6050();
}

void Setup_MPU6050()
{
    Wire.begin();
   // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    //Initialize DMP in MPU 6050
    Setup_MPU6050_DMP();
}

//Setup MPU 6050 DMP
void Setup_MPU6050_DMP()
{
  
     //DMP Initialization
   devStatus = accelgyro.dmpInitialize();
   accelgyro.setXGyroOffset(220);
   accelgyro.setXGyroOffset(76);
   accelgyro.setXGyroOffset(-85); 
   accelgyro.setXGyroOffset(1788);  
   
   if(devStatus == 0){
    
      accelgyro.setDMPEnabled(true);
      pinMode(PUSH2,INPUT_PULLUP);    
      attachInterrupt(PUSH2, dmpDataReady, RISING);
      mpuIntStatus = accelgyro.getIntStatus();
      dmpReady = true;
      packetSize = accelgyro.dmpGetFIFOPacketSize();
     
    }
 
}

long range_time;

void loop()
{

    Update_MPU6050_DMP();

    nh.spinOnce();    
}

void Update_MPU6050_DMP()
{
  
  //DMP Processing

    if (!dmpReady) return;
    
    while (!mpuInterrupt && fifoCount < packetSize) {
      ;    
    }

    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();
    
    //get current FIFO count
    fifoCount = accelgyro.getFIFOCount();
    
    
    if ((mpuIntStatus & 0x10) || fifoCount > 512) {
        // reset so we can continue cleanly
        accelgyro.resetFIFO();
    }

    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

        // read a packet from FIFO
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


         if ( millis() >= range_time ){

            imu_msg.orientation.x = q.x;
            imu_msg.orientation.y = q.y;
            imu_msg.orientation.z = q.z;
            imu_msg.orientation.w = q.w;

            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;

            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;
            
            imu_msg.header.stamp = nh.now();
            pub_imu.publish(&imu_msg);

            range_time =  millis() + 60;
          }     
    }
}


