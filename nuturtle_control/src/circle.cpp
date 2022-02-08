/// \file
/// \brief Node makes the Turtlebot move in a circle
///
/// PARAMETERS:
///     frequency (double): Determines the amount of times a second the loop runs
/// PUBLISHERS:
///     cmd_vel_pub (geometry_msgs/Twist): Publish to cmd_vel to have the robot move
/// SERVICES:
///     control_service (nuturtle_control/Control): Go in a circle based on a angular velocity and radius
///     reverse_service (std_srvs/Empty): Stop the robot 
///     stop_service (std_srvs/Empty): Make the robot go in reverse

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

/// \brief Create a circle based on input from a user

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
            
            main_loop();
        }

        /// \brief Make the robot go in a circle
        ///
        /// \param data - Make a circle based on a radius and angular velocity
        /// \returns response - true 
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

        /// \brief Make the robot go in reverse 
        ///
        /// \param Empty - Empty
        /// \returns response - true 
        bool reverse_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& )
        {
            stop_flag = 0;
            twist.linear.x = -x_dot;
            twist.angular.z = -thetadot;
            twist.linear.y = 0;
            return true;
        }

        /// \brief Stop the robot and stop the node from publishing twist values
        ///
        /// \param Empty - Empty
        /// \returns response - true 
        bool stop_callback(std_srvs::Empty::Request& , std_srvs::Empty::Response& )
        {
            stop_flag = 1;
            twist.linear.x = 0;
            twist.angular.z = 0;
            twist.linear.y = 0;

            // cmd_vel_pub.publish(twist);
            return true;
        }

        /// \brief A timer that updates the simulation
        ///
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

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "circle");
    circle node;
    ros::spin();
    return 0;
}