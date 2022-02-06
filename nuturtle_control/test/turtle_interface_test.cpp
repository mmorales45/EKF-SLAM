#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>

geometry_msgs::Twist cmd_vel;
nuturtlebot_msgs::WheelCommands wheel_cmd;

void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands & msg)
{
  wheel_cmd = msg;
  
}

TEST_CASE("Purely Translational Wheel_cmds", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; // this initializes time

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("/wheel_cmd", 1, wheel_cmd_callback);

  geometry_msgs::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 1.0;

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
  CHECK(wheel_cmd.left_velocity == 27);
  CHECK(wheel_cmd.right_velocity == 32);
}


TEST_CASE("Purely Rotational Wheel_cmds", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; // this initializes time

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("/wheel_cmd", 1, wheel_cmd_callback);

  geometry_msgs::Twist twist;
  twist.linear.x = 1.0;
  // twist.angular.z = 1.0;

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
  CHECK(wheel_cmd.left_velocity == 30);
  CHECK(wheel_cmd.right_velocity == 30);
}

TEST_CASE("Sensor Encoder data", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; // this initializes time

  ros::Publisher sense_pub = nh.advertise<nuturtlebot_msgs::SensorData>("/sensor_data",1);
  ros::Subscriber wheel_cmd_sub = nh.subscribe("/joint_states", 1, wheel_cmd_callback);

  nuturtlebot_msgs::SensorData sense_data;
  sense_data.left_encoder = 10;
  sense_data.right_encoder = 10;

  ros::Rate r(500);
  while(1){
    i++;
    sense_pub.publish(sense_data);
    ros::spinOnce();
    if (i > 1000){
      break;
    }
  }  
  CHECK(wheel_cmd.left_velocity == 30);
  CHECK(wheel_cmd.right_velocity == 30);
}