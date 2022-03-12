/// \file
/// \brief Node that detects landmarks
///
/// PARAMETERS:
///     
/// PUBLISHERS:
///     
/// SUBSCRIBERS:
///     
/// SERVICES:
///     


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
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <cmath>

#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nav_msgs/Odometry.h>
#include <nuturtle_control/SetPose.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <armadillo>
#include "nuslam/nuslam.hpp"

/// \brief Create and update the odometry between the world and blue turtlebot

class Landmarks
{
    public:
        Landmarks() {
            
            obstacles_sub = nh.subscribe("/nusim/obstacles/Fake_markerArray",10,&Landmarks::obstacle_callback,this);
            timer = nh.createTimer(ros::Duration(1.0/500.0), &Landmarks::main_loop, this);
        }


        /// \brief Callback for the fake sensor data topic.
        ///
        /// \param fake_obstacles - Position of the noisy obstacles relative to the robot.
        void obstacle_callback(const visualization_msgs::MarkerArray & fake_obstacles)
        {
            int size_laser = fake_obstacles.size();
        }

       
        /// \brief A timer that continuosly publishes odometry, creates transforms, and updates the DiffDrive class members
        ///
        void main_loop(const ros::TimerEvent &)
        {
            
        }

    private:
    //create private variables
    ros::NodeHandle nh;
    ros::Subscriber obstacles_sub;
    ros::Timer timer;
    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "LANDMARKS");
    Landmarks node;
    ros::spin();
    return 0;
}