# TivaC_MoblieCar

## ROS Topic: [ROS common_msgs](http://wiki.ros.org/common_msgs)
* Publish : sensor_msgs::Range (IR & Ultrasound) , nav_msgs::Odoemetry  
* Subscribe : geometry_msgs::Twist  

## ROS local & remote connect:
* ssh **name**@**IP**
* vim /home/.bashrc
* In local:  
    export MY_IP=**IP**  
    export ROS_IP=$MY_IP  
    export ROS_MASTER_URI="http://"$ROS_IP":11311"
* In remote:  
    export ROS_MASTER_URI="http://**IP**:11311"
                     
## Tiva TM4C1294 Pin Table: 
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/23633173_1720082644676737_2065439414_o.jpg)

## 3.3 to 5 voltage level shifter direction:
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/3.3V-5V-Logic-Level-Converter-5.jpg)
