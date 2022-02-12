#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <sensor_msgs/JointState.h>

//global variables
geometry_msgs::Twist cmd_vel;
nuturtlebot_msgs::WheelCommands wheel_cmd;
sensor_msgs::JointState jointStates;
// subscriber callbacks
void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands & msg)
{
  wheel_cmd = msg;
  
}

void js_callback(const sensor_msgs::JointState & js)
{
  jointStates = js;
  
}

TEST_CASE("Purely Translational Wheel_cmds", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; 
  // init publishers and subscribers
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  ros::Subscriber js_sub = nh.subscribe("/wheel_cmd", 1, wheel_cmd_callback);
  // create twist
  geometry_msgs::Twist twist;
  twist.linear.x = 0.1;
  twist.angular.z = 0.1;
  // run for 2 seconds
  ros::Rate r(500);
  while(1){
    i++;
    cmd_vel_pub.publish(twist);
    ros::spinOnce();
    r.sleep();
    if (i > 1000){
      break;
    }
  }  
  CHECK(wheel_cmd.left_velocity == 116);
  CHECK(wheel_cmd.right_velocity == 136);
}


TEST_CASE("Purely Rotational Wheel_cmds", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; 

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("/wheel_cmd", 1, wheel_cmd_callback);
  // create twist
  geometry_msgs::Twist twist;
  twist.angular.z = 0.1;
  // run for 2 seconds
  ros::Rate r(500);
  while(1){
    i++;
    cmd_vel_pub.publish(twist);
    ros::spinOnce();
    r.sleep();
    if (i > 1000){
      break;
    }
  }  
  CHECK(wheel_cmd.left_velocity == -10);
  CHECK(wheel_cmd.right_velocity == 10);
}

TEST_CASE("Sensor Encoder data", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh;

  ros::Publisher sense_pub = nh.advertise<nuturtlebot_msgs::SensorData>("/sensor_data",1);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("/joint_states", 1, js_callback);
  // create sensorData data type variable
  nuturtlebot_msgs::SensorData sense_data;
  sense_data.left_encoder = 10;
  sense_data.right_encoder = 10;
  // run for 1 second
  ros::Rate r(500);
  while(1){
    i++;
    sense_pub.publish(sense_data);
    ros::spinOnce();
    r.sleep();
    if (i > 500){
      break;
    }
  }  
  CHECK(jointStates.position[0] == 0.0153398078);
  CHECK(jointStates.position[1] == 0.0153398078);
}