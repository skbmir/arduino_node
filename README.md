# arduino_node

ROS node for Arduino

Depends on:
* [BNO055 driver for Arduino by Adafruit](https://github.com/adafruit/Adafruit_BNO055)
* [Adafruit Unified Sensor Library](https://github.com/adafruit/Adafruit_Sensor)
* [rosserial_arduino](http://wiki.ros.org/rosserial_arduino)

Command processing is done in cmd_vel_cb function. This code is written to control 2 DC motors and steering servo.


#### Subscribed topics
------
* cmd_vel (geometry_msgs/Twist) - velosity and steer commands from local planner

#### Published topics
------
* imu_data (sensor_msgs/Imu) - data from BNO055 9 DOF IMU
