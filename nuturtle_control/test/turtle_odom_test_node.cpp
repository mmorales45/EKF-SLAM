#include <catch_ros/catch.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtle_control/SetPose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
 #include <geometry_msgs/TransformStamped.h>

geometry_msgs::Twist cmd_vel;
nuturtlebot_msgs::WheelCommands wheel_cmd;
nav_msgs::Odometry odom_var;

void odom_callback(const nav_msgs::Odometry& data)
{
  odom_var = data;
  
}

TEST_CASE("Transform Odom and base_footprint", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; // this initializes time
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  double x,y;
  ros::Rate r(100);
  while(1){
    i++;
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("odom", "blue-base_footprint",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    x = transformStamped.transform.translation.x;
    y = transformStamped.transform.translation.y;
    ros::spinOnce();
    r.sleep();
    if (i > 1000){
      break;
    }
  }

CHECK(x == 0.0);
CHECK(y == 0.0);

}

TEST_CASE("Set Pose", "[nuturtle_control]") {
  int i;
  ros::NodeHandle nh; // this initializes time
  ros::ServiceClient set_pose_srv = nh.serviceClient<nuturtle_control::SetPose>("/set_pose");
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_callback);

  ros::Rate r(100);
  while(1){
    i++;
    // ros::service::waitForService("/set_pose");
    nuturtle_control::SetPose set_pose_msg;
    int flag = 0;
    set_pose_msg.request.x = 1.0;
    set_pose_msg.request.y = 1.0;
    set_pose_msg.request.theta = 0;
    set_pose_srv.call(set_pose_msg);    
    flag =1;
    ros::spinOnce();
    r.sleep();
    if (i > 1000){
      break;
    }
  }
  CHECK(odom_var.pose.pose.position.x == 1.0);
  CHECK(odom_var.pose.pose.position.y == 1.0);
  CHECK(odom_var.pose.pose.position.z == 0.0);
  CHECK(odom_var.pose.pose.orientation.x == 0.0);
  CHECK(odom_var.pose.pose.orientation.y == 0.0);
  CHECK(odom_var.pose.pose.orientation.z == 0.0);
  CHECK(odom_var.pose.pose.orientation.z == 0.0);
}

