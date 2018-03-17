# TivaC_MoblieCar

## ROS Topic: [ROS common_msgs](http://wiki.ros.org/common_msgs)
* Publish : sensor_msgs::Range (IR & Ultrasound) , nav_msgs::Odoemetry  
* Subscribe : geometry_msgs::Twist  

## ROS local & remote connect:
* IP : ifconfig
* ttyACM : dmesg | grep tty
* make sure open ssh : sudo service ssh start ; netstat -a | grep ssh
* ssh **LOCAL_NAME**@**LOCAL_IP**
* vim /home/**USER**/.bashrc
* In local:  
    export MY_IP=**LOCAL_IP**  
    export ROS_IP=$MY_IP  
    export ROS_MASTER_URI="http://**LOCAL_IP**:11311"
* In remote:  
    export ROS_IP=**REMOTE_IP**  
    export ROS_MASTER_URI="http://**LOCAL_IP**:11311"
  
## EXECUTE:
* Enable RPLidar:
    sudo chmod 666 /dev/ttyUSB0
    roslaunch rplidar_ros view_rplidar.launch
* Enable TivaC:
    sudo chmod 666 /dev/ttyACM0 (or ttyACM1)
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
* TF (frame: odom -> base_link -> laser):
    check tf tree:  rosrun tf view_frames
* Mapping:
    rosrun gmapping slam_gmapping scan:=scan
  
## Tiva TM4C1294 Pin Table(3.3v): 
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/23633173_1720082644676737_2065439414_o.jpg)

## 3.3 to 5 voltage level shifter direction:
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/3.3V-5V-Logic-Level-Converter-5.jpg)

## Motor Pin Table(encoder-5v): 
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/motor_pin.JPG) 

## Motor driver Pin Table(driver-5v & output for motor-12v):
![image](https://github.com/glitter2626/TivaC_MoblieCar/blob/master/motordriver_pin.JPG)  
        ***(from : Learning Robotics using Python)***

