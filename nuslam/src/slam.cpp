/// \file
/// \brief Node that updates the odometry between the odom and blue turtlebot3
///
/// PARAMETERS:
///     robot (std::vector<double>): The initial x,y,theta of the turtlebot
///     odom_id (std::string): The name of the odom frame
///     body_id (std::string): Name of the body id
///     left_wheel_joint (std::string): Name of the left wheel joint
///     right_wheel_joint (std::string): Name of the right wheel joint
/// PUBLISHERS:
///     odom_pub (nav_msgs/Odometry): Update the odometry based on the Turtlebot's data
/// SUBSCRIBERS:
///     joint_state_sub (sensor_msgs/JointState): Receive the wheels' position and velocities to updates then odometry
/// SERVICES:
///     set_pose_service (nuturtle_control/SetPose): Set the location of the blue Turtlebot by specifying a x,y,theta

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

class odometry
{
    public:
        odometry() {
            loadParams();
            rate = 500;
            //set frame ids and initial position of robot
            odom.header.frame_id = odom_id;
            odom.child_frame_id = body_id;
            transformStamped.header.frame_id = odom_id;
            transformStamped.child_frame_id = body_id;
            nh.getParam("/robot",robot_coords);
            current_config.x = robot_coords[0];
            current_config.y = robot_coords[1];
            current_config.theta = robot_coords[2];
            DiffDrive = turtlelib::DiffDrive(current_config);
            
            joint_state_sub = nh.subscribe("/joint_states",10,&odometry::js_callback,this);
            obstacles_sub = nh.subscribe("/nusim/obstacles/Fake_markerArray",10,&odometry::obstacle_callback,this);
            odom_pub = nh.advertise<nav_msgs::Odometry>("odom",100);
            path_pub  = nh.advertise<nav_msgs::Path>("odometry/path", 10, true);
            set_pose_service = nh.advertiseService("set_pose", &odometry::set_pose_callback, this);

            EKFilter = nuslam::KalmanFilter();
            c = arma::mat(3*(3),1,arma::fill::zeros);
            initial_flag = 0;

            timer = nh.createTimer(ros::Duration(1.0/rate), &odometry::main_loop, this);
        }

        /// \brief Get the updated information of the Turtlebots through their wheel positions and velocities
        ///
        /// \param js - Has the updated positions and velocities for both the left and right link
        void js_callback(const sensor_msgs::JointState & js)
        {
            new_vel.left_vel = js.velocity[0];
            new_vel.right_vel = js.velocity[1];
            new_angles.left_angle = js.position[0];
            new_angles.right_angle = js.position[1];
        }

        /// \brief Teleport the Turtlebot3 to a specified location
        ///
        /// \param data - x,y, and theta values for where the robot should be moved to
        /// \returns response - true 
        bool set_pose_callback(nuturtle_control::SetPose::Request& data, nuturtle_control::SetPose::Request& )
        {
            current_config.theta = data.theta;
            current_config.x = data.x;
            current_config.y = data.y;
            // DiffDrive = turtlelib::DiffDrive(current_config);
            return true;
        }

        void make_path()
        {
            current_path.header.stamp = ros::Time::now();
            current_path.header.frame_id = "map";
            path_pose.header.stamp = ros::Time::now();
            path_pose.header.frame_id = "blue-base_footprint";
            path_pose.pose.position.x = current_config.x;
            path_pose.pose.position.y = current_config.y;
            path_pose.pose.orientation.x = q.x();
            path_pose.pose.orientation.y = q.y();
            path_pose.pose.orientation.z = q.z();
            path_pose.pose.orientation.w = q.w();
            current_path.poses.push_back(path_pose);
            path_pub.publish(current_path);
        }
        /// \brief Loads the paramters from the parameter server to be used in the node
        ///
        void loadParams()
        {
            if(!nh.getParam("/odom_id",odom_id)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for odom");
                ros::shutdown();
            }
            else{
                nh.getParam("/odom_id",odom_id);
            }
            if(!nh.getParam("/body_id",body_id)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for body");
                ros::shutdown();
            }
            else{
                nh.getParam("/body_id",body_id);
            }
            if(!nh.getParam("/left_wheel_joint",left_wheel_joint)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for left wheel");
                ros::shutdown();
            }
            else{
                nh.getParam("/left_wheel_joint",left_wheel_joint);
            }
            if(!nh.getParam("/right_wheel_joint",right_wheel_joint)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for right wheel");
                ros::shutdown();
            }
            else{
                nh.getParam("/right_wheel_joint",right_wheel_joint);
            }
        }
        void obstacle_callback(const visualization_msgs::MarkerArray & fake_obstacles)
        {
            // ROS_WARN("id: %d",fake_obstacles.markers.size());
            int val = int (fake_obstacles.markers.size());
            z_values = arma::mat(2*(val),1,arma::fill::zeros);
            
            for (int t = 0;t<val;t++)
            {
                double x_i = fake_obstacles.markers[t].pose.position.x;
                double y_i = fake_obstacles.markers[t].pose.position.y;
                turtlelib::Vector2D coord_vect;
                coord_vect.x = x_i;
                coord_vect.y = y_i;
                double radius_i = sqrt(pow(x_i,2)+pow(y_i,2));
                double phi_i = turtlelib::normalize_angle(atan2(y_i,x_i));
                // ROS_WARN("id: %f obs %d",radius_i,t);
                // ROS_WARN("id: %f",phi_i);
                z_values(2*t,0) = radius_i;
                z_values((2*t)+1,0) = phi_i;
                // ROS_WARN("id: %d",fake_obstacles.markers[t].id);
                if (initial_flag == 0)
                {
                    EKFilter.Landmark_Initialization(t,coord_vect);
                }

            }   

            initial_flag = 1;

            c = EKFilter.predict(twist);
            ROS_INFO_STREAM(c);

            arma::mat b = EKFilter.update(val,z_values); 
            // ROS_INFO_STREAM(b);

            
        }
        
        /// \brief A timer that continuosly publishes odometry, creates transforms, and updates the DiffDrive class members
        ///
        void main_loop(const ros::TimerEvent &)
        {
            make_path();
            twist = DiffDrive.Twist_from_wheelVel(new_vel);
            //update configuration based on forward kinematics
            current_config = DiffDrive.forward_Kinematics(new_angles,current_config);
            //make the new angles the old ones
            old_angles.left_angle = new_angles.left_angle;
            old_angles.right_angle = new_angles.right_angle;
            //publish odom and transform
            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = c(1,0);
            odom.pose.pose.position.y = c(2,0);
            //create quaternion from theta
            q.setRPY(0, 0, current_config.theta);
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();

            odom.twist.twist.angular.z = twist.theta_dot;
            odom.twist.twist.linear.x = twist.x_dot;
            odom.twist.twist.linear.y = twist.y_dot;

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = c(1,0);
            transformStamped.transform.translation.y = c(2,0);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            odom_pub.publish(odom);
            broadcaster.sendTransform(transformStamped);


            

        }
    private:
    //create private variables
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub;
    ros::Subscriber obstacles_sub;
    ros::Publisher odom_pub;
    ros::Publisher path_pub;
    ros::ServiceServer set_pose_service;

    std::string odom_id;
    std::string body_id;
    std::string left_wheel_joint;
    std::string right_wheel_joint;

    turtlelib::phi_angles new_angles;
    turtlelib::phi_angles old_angles;

    turtlelib::speed new_vel;
    turtlelib::Twist2D twist;
    turtlelib::DiffDrive DiffDrive;
    turtlelib::config current_config;

    nav_msgs::Odometry odom;
    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    ros::Timer timer;
    std::vector<double> robot_coords;

    double rate;
    tf2::Quaternion q;
    nav_msgs::Path current_path;
    geometry_msgs::PoseStamped path_pose;

    arma::mat z_values;
    nuslam::KalmanFilter EKFilter;
    arma::mat c;
    int initial_flag;
    
    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry");
    odometry node;
    ros::spin();
    return 0;
}