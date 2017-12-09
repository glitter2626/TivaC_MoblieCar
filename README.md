# TivaC_MoblieCar

## ROS Topic: [ROS common_msgs](http://wiki.ros.org/common_msgs)
* Publish : sensor_msgs::Range (IR & Ultrasound) , nav_msgs::Odoemetry  
* Subscribe : geometry_msgs::Twist  

## ROS local & remote connect:
* ssh **LOCAL_NAME**@**LOCAL_IP**
* vim /home/**USER**/.bashrc
* In local:  
    export MY_IP=**LOCAL_IP**  
    export ROS_IP=$MY_IP  
    export ROS_MASTER_URI="http://"$ROS_IP":11311"
* In remote:  
    export ROS_MASTER_URI="http://**LOCAL_IP**:11311"
                     
## Tiva TM4C1294 Pin Table: 
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/23633173_1720082644676737_2065439414_o.jpg)

## 3.3 to 5 voltage level shifter direction:
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/3.3V-5V-Logic-Level-Converter-5.jpg)

## Motor Pin Table: 
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/motor_pin.JPG) 

## Motor driver Pin Table:
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/motordriver_pin.JPG) 

