#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <iostream>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cstdlib>
#include <tf/transform_broadcaster.h>


#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nav_msgs/Odometry.h>
#include <nuturtle_control/Control.h>

class circle
{
    public:
        circle() {
            rate = 100;
            stop_flag = 0;
            nh.getParam("circle/frequency",frequency);
            twist.angular.z =0;
            twist.linear.x =0;
            control_service = nh.advertiseService("control", &circle::control_callback, this);
            reverse_service = nh.advertiseService("reverse", &circle::reverse_callback, this);
            stop_service = nh.advertiseService("stop", &circle::stop_callback, this);
            cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
            
            // timer = nh.createTimer(ros::Duration(1/100), &circle::main_loop, this);
            main_loop();
        }
          
        bool control_callback(nuturtle_control::Control::Request& data, nuturtle_control::Control::Response& )
        {  
            stop_flag = 0;
            thetadot = data.angular_velocity;
            radius = data.radius;
            x_dot = thetadot*radius;

            twist.linear.x = x_dot;
            twist.angular.z = thetadot;
            twist.linear.y = 0;

            return true;
        }

        bool reverse_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& )
        {
            twist.linear.x = -x_dot;
            twist.angular.z = -thetadot;
            twist.linear.y = 0;
            return true;
        }

        bool stop_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& )
        {
            stop_flag = 1;
            twist.linear.x = 0;
            twist.angular.z = 0;
            twist.linear.y = 0;

            // cmd_vel_pub.publish(twist);
            return true;
        }

        // void main_loop(const ros::TimerEvent &)
        // {
        //     if(stop_flag == 0){
        //         cmd_vel_pub.publish(twist);
        //     }
        //     else if(stop_flag == 1){
        //         cmd_vel_pub.publish(twist);
        //         stop_flag = 2;
        //     }
        // }
        void main_loop()
        {
            ros::Rate r(frequency);
            while(ros::ok()){
                if(stop_flag == 0){
                    cmd_vel_pub.publish(twist);
                }
                else if(stop_flag == 1){
                    cmd_vel_pub.publish(twist);
                    stop_flag = 2;
                }
                ros::spinOnce();
                r.sleep();
            }
        }
        
    private:
    ros::NodeHandle nh;
    ros::ServiceServer control_service;
    ros::ServiceServer reverse_service;
    ros::ServiceServer stop_service;
    ros::Publisher cmd_vel_pub;

    geometry_msgs::Twist twist;
    double thetadot;
    double radius;
    double x_dot;
    ros::Timer timer;
    double frequency;

    int stop_flag;

    double rate;
    
    
    
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "circle");
    circle node;
    ros::spin();
    return 0;
}