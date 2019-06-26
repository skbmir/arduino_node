#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#define BNO055_SAMPLERATE_DELAY_MS (20)
#define SERVO_PWM_PIN 8
#define ZERO_STEER 80
#define MTR1_SPD_PIN 5
#define MTR1_DIR_PIN 4
#define MTR2_SPD_PIN 6
#define MTR2_DIR_PIN 7
#define MAX_STEER 10.0
#define MAX_SPEED 255.0

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

Servo servo;

ros::NodeHandle node;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_vel_cb);

void setup()
{
  servo.attach(SERVO_PWM_PIN);
  servo.write(ZERO_STEER);
  
  // initialize pins from 4 to 8
  // refer to #defines at the to of this file
  // for individual pin purpose
  for (int i = 4; i < 8; i++)
    pinMode(i, OUTPUT);
  
  node.initNode();
  node.advertise(imu_pub);
  node.subscribe(cmd_sub);
  
  if (!bno.begin())
  {
    Serial.print("BNO055 not detected.");
    while(1);
  }
  
  delay(1000); // BNO055 initialization delay, roughly 1 second
  bno.setExtCrystalUse(true);
}

void loop()
{
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angular = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  imu_msg.header.stamp = node.now();
  imu_msg.header.frame_id = "base_imu";
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();
  imu_msg.angular_velocity.x = angular.x();
  imu_msg.angular_velocity.y = angular.y();
  imu_msg.angular_velocity.z = angular.z();
  imu_msg.linear_acceleration.x = linear.x();
  imu_msg.linear_acceleration.y = linear.y();
  imu_msg.linear_acceleration.z = linear.z();
  imu_pub.publish(&imu_msg);
  
  node.spinOnce();
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


// callback function for processing commands from high level ROS nodes
void cmd_vel_cb(const geometry_msgs::Twist& cmd_vel)
{
  double speed = cmd_vel.linear.x;
  speed = (int)(speed * MAX_SPEED);
  double steer = cmd_vel.angular.z;
  steer = (int)(steer * MAX_STEER);
  steer = steer + ZERO_STEER;
  
  if (steer > 90)
    steer = 90;
  else if (steer < 70)
    steer = 70;
  
  servo.write(steer);
  
  if (speed >= 0 && speed <= 255)
  {
    digitalWrite(MTR1_DIR_PIN, HIGH);
    digitalWrite(MTR2_DIR_PIN, LOW);
    analogWrite(MTR1_SPD_PIN, speed);
    analogWrite(MTR2_SPD_PIN, speed);
  }
  else if (speed < 0 && speed > -255)
  {
    speed = -speed;
    digitalWrite(MTR1_DIR_PIN, LOW);
    digitalWrite(MTR2_DIR_PIN, HIGH);
    analogWrite(MTR1_SPD_PIN, speed);
    analogWrite(MTR2_SPD_PIN, speed);
  }
}
