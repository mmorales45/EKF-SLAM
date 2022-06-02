/// \file
/// \brief Node that applies EKF filter to map robot's and obstacle's locations.
///
/// PARAMETERS:
///     robot (std::vector<double>): The initial x,y,theta of the turtlebot
///     odom_id (std::string): The name of the odom frame
///     body_id (std::string): Name of the body id
///     left_wheel_joint (std::string): Name of the left wheel joint
///     right_wheel_joint (std::string): Name of the right wheel joint
/// PUBLISHERS:
///     odom_pub (nav_msgs/Odometry): Update the odometry based on the Turtlebot's data
///     path_pub (nav_msgs/Path): Create the path of the blue robot.
///     green_path_pub (nav_msgs/Path): Create the path of the green robot.
///     fake_marker_pub (visualization_msgs/MarkerArray): Publish Markers based on SLAM data.
/// SUBSCRIBERS:
///     joint_state_sub (sensor_msgs/JointState): Receive the wheels' position and velocities to updates then odometry
///     obstacles_sub visualization_msgs/MarkerArray): Receive the location of the markers based on the red/real robot.
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

class SLAM
{
    public:
        SLAM() {
            loadParams();
            rate = 500;
            //set frame ids and initial position of robot
            odom.header.frame_id = odom_id;
            odom.child_frame_id = "green-base_footprint";
            transformStamped.header.frame_id = "world";// for blue robot
            transformStamped.child_frame_id = "blue-base_footprint";// for blue robot
            transformStamped_mo.header.frame_id = "map";
            transformStamped_mo.child_frame_id = odom_id;// for green robot
            transformStamped_green.header.frame_id = odom_id;// for green robot
            transformStamped_green.child_frame_id = "green-base_footprint";// for green robot
            nh.getParam("/robot",robot_coords);
            current_config.x = robot_coords.at(0);
            current_config.y = robot_coords.at(1);
            current_config.theta = robot_coords.at(2);
            DiffDrive = turtlelib::DiffDrive(current_config);
            
            joint_state_sub = nh.subscribe("/joint_states",10,&SLAM::js_callback,this);
            obstacles_sub = nh.subscribe("/points/Obstacles",10,&SLAM::obstacle_callback,this);
            odom_pub = nh.advertise<nav_msgs::Odometry>("odom",100);
            path_pub  = nh.advertise<nav_msgs::Path>("SLAM/path", 10, true);
            green_path_pub = nh.advertise<nav_msgs::Path>("SLAM/green_path", 10, true);
            set_pose_service = nh.advertiseService("set_pose", &SLAM::set_pose_callback, this);
            fake_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nuslam/obstacles/Fake_markerArray", 1, true);

            EKF_FLAG = 0;
            initial_flag = 0;
            state = arma::mat(3,1,arma::fill::zeros);
            timer = nh.createTimer(ros::Duration(1.0/rate), &SLAM::main_loop, this);
            timer_green = nh.createTimer(ros::Duration(1.0/5.0), &SLAM::green_loop, this);
            landmark_counter= 0;
        }

        /// \brief Get the updated information of the Turtlebots through their wheel positions and velocities
        ///
        /// \param js - Has the updated positions and velocities for both the left and right link
        void js_callback(const sensor_msgs::JointState & js)
        {
            new_vel.left_vel = js.velocity.at(0);
            new_vel.right_vel = js.velocity.at(1);
            new_angles.left_angle = js.position.at(0);
            new_angles.right_angle = js.position.at(1);
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
            return true;
        }

        /// \brief Create path for Blue Robot
        void make_path()
        {
            current_path.header.stamp = ros::Time::now();
            current_path.header.frame_id = "world";
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
            nh.getParam("/radius",radius);

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
            nh.getParam("/max_range",max_range);
        }

        /// \brief Callback for the fake sensor data topic, main slam implementation. 
        ///
        /// \param fake_obstacles - Position of the noisy obstacles relative to the robot.
        void obstacle_callback(const visualization_msgs::MarkerArray & fake_obstacles)
        {
            val = int (fake_obstacles.markers.size());
            ROS_INFO_STREAM("NUM OF OBSTACLES");
            ROS_INFO_STREAM(val);
            ROS_INFO_STREAM("NUM OF OBSTACLES");
            z_values = arma::mat(2*(val),1,arma::fill::zeros);
            if (EKF_FLAG == 0)
            {
                EKFilter = nuslam::KalmanFilter(100,100.0,0.1);
                EKF_FLAG = 1;
            }
            
            for (int t = 0;t<val;t++)
            {
                
                double x_i = fake_obstacles.markers[t].pose.position.x;
                double y_i = fake_obstacles.markers[t].pose.position.y;
                turtlelib::Vector2D current_land;
                current_land = {x_i,y_i};
                current_land = T_MB(current_land);
                double radius_i = sqrt(pow(x_i,2)+pow(y_i,2));
                double phi_i = turtlelib::normalize_angle(atan2(y_i,x_i));
                
                arma::mat init_z(2,1,arma::fill::zeros);
                init_z(0,0) = radius_i;
                init_z(1,0) = phi_i;

                z_values(2*t,0) = radius_i;
                z_values((2*t)+1,0) = phi_i;

                if (known_landmarks.size()==0)
                {
                    known_landmarks.push_back(current_land);
                    EKFilter.Landmark_Initialization(landmark_counter,init_z);
                    landmark_counter++;

                }
                else
                {
                    bool new_landmark = EKFilter.CheckLandmarks(known_landmarks,current_land);
                    if (new_landmark && landmark_counter<99)
                    {
                        known_landmarks.push_back(current_land);
                        EKFilter.Landmark_Initialization(landmark_counter,init_z);
                        landmark_counter++;
                    }
                    else
                    {
                        ;
                    }

                }

                
            }   
            
            for (int ii=0; ii<int(known_landmarks.size());ii++)
            {
                ROS_INFO_STREAM(known_landmarks.at(ii));
            }
            ROS_INFO_STREAM("SIZE OF LANDMARKS");
            ROS_INFO_STREAM(known_landmarks.size());
            
            initial_flag = 1;
            EKFilter.predict(twist);

            state = EKFilter.update(val,z_values); 
            make_fake_obstacles();
            EKFilter.ResetN();

            
        }
        
        /// \brief Create path for Green Robot/SLAM
        void create_green_path()
        {
            green_current_path.header.stamp = ros::Time::now();
            green_current_path.header.frame_id = "world";
            green_path_pose.header.stamp = ros::Time::now();
            green_path_pose.header.frame_id = "green-base_footprint";
            green_path_pose.pose.position.x = new_vect_MB.x;
            green_path_pose.pose.position.y = new_vect_MB.y;
            q_MB.setRPY(0, 0, new_theta_MB);
            green_path_pose.pose.orientation.x = q_MB.x();
            green_path_pose.pose.orientation.y = q_MB.y();
            green_path_pose.pose.orientation.z = q_MB.z();
            green_path_pose.pose.orientation.w = q_MB.w();
            green_current_path.poses.push_back(green_path_pose);
            green_path_pub.publish(green_current_path);
        }

        /// \brief Create the obstacles based on the SLAM information. 
        void make_fake_obstacles()
        {            
            fake_marker.markers.resize(val);
            for (int i = 0;i<val;i++)
            {
                turtlelib::Vector2D obstacle_B_MARK;
                obstacle_B_MARK.x = state(3+2*i,0);
                obstacle_B_MARK.y = state(4+2*i,0);


                fake_marker.markers[i].header.stamp = ros::Time();
                fake_marker.markers[i].header.frame_id = "world";
                shape = visualization_msgs::Marker::CYLINDER;
                fake_marker.markers[i].type = shape;
                fake_marker.markers[i].ns = "slam_fake_obstacles";

                fake_marker.markers[i].id = i;

                fake_marker.markers[i].pose.position.x = obstacle_B_MARK.x;
                fake_marker.markers[i].pose.position.y = obstacle_B_MARK.y;
                fake_marker.markers[i].pose.position.z = 0.125;
                fake_marker.markers[i].pose.orientation.x = 0.0;
                fake_marker.markers[i].pose.orientation.y = 0.0;
                fake_marker.markers[i].pose.orientation.z = 0.0;
                fake_marker.markers[i].pose.orientation.w = 1.0;

                fake_marker.markers[i].scale.x = (2*radius);
                fake_marker.markers[i].scale.y = (2*radius);
                fake_marker.markers[i].scale.z = 0.25;
                
                fake_marker.markers[i].color.r = 0.0;
                fake_marker.markers[i].color.g = 1.0;
                fake_marker.markers[i].color.b = 0.0;
                fake_marker.markers[i].color.a = 1.0;

                double distance = sqrt(pow(obstacle_B_MARK.x-current_config.x,2)+pow(obstacle_B_MARK.y-current_config.y,2));
                if (distance < max_range){
                    fake_marker.markers[i].action = visualization_msgs::Marker::ADD;
                }
                else{
                    fake_marker.markers[i].action = visualization_msgs::Marker::DELETE;
                }

            }
            fake_marker_pub.publish(fake_marker);
            
        }


        /// \brief A timer that continuosly publishes odometry, creates transforms, and updates the DiffDrive class members
        ///
        void main_loop(const ros::TimerEvent &)
        {
            twist = DiffDrive.Twist_from_wheelVel(new_vel);
            //update configuration based on forward kinematics
            current_config = DiffDrive.forward_Kinematics(new_angles,current_config);
            
            //create transform from odom to base
            turtlelib::Vector2D vect_OB;
            vect_OB.x = current_config.x;
            vect_OB.y = current_config.y;
            double thetaOB = current_config.theta;
            T_OB = turtlelib::Transform2D(vect_OB,thetaOB);
            turtlelib::Vector2D new_vect_OB = T_OB.translation();
            double new_theta_OB = T_OB.rotation();

            //create transform from map to base
            turtlelib::Vector2D vect_MB;
            vect_MB.x = state(1,0);
            vect_MB.y = state(2,0);
            double theta_MB = state(0,0);
            T_MB = turtlelib::Transform2D(vect_MB,theta_MB);
            new_vect_MB = T_MB.translation();
            new_theta_MB = T_MB.rotation();

            //find transform from map to odom
            T_MO = T_MB * T_OB.inv();
            turtlelib::Vector2D new_vect_MO = T_MO.translation();
            double new_theta_MO = T_MO.rotation();

            //make the new angles the old ones
            old_angles.left_angle = new_angles.left_angle;
            old_angles.right_angle = new_angles.right_angle;
            //publish odom and transform
            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = new_vect_OB.x;
            odom.pose.pose.position.y = new_vect_OB.y;
            //create quaternion from theta
            q_OB.setRPY(0, 0, new_theta_OB);
            odom.pose.pose.orientation.x = q_OB.x();
            odom.pose.pose.orientation.y = q_OB.y();
            odom.pose.pose.orientation.z = q_OB.z();
            odom.pose.pose.orientation.w = q_OB.w();

            odom.twist.twist.angular.z = twist.theta_dot;
            odom.twist.twist.linear.x = twist.x_dot;
            odom.twist.twist.linear.y = twist.y_dot;

            q.setRPY(0, 0, current_config.theta);
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = current_config.x;
            transformStamped.transform.translation.y = current_config.y;
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            transformStamped_mo.header.stamp = ros::Time::now();
            transformStamped_mo.transform.translation.x = new_vect_MO.x;
            transformStamped_mo.transform.translation.y = new_vect_MO.y;
            q_MO.setRPY(0, 0, new_theta_MO);
            transformStamped_mo.transform.rotation.x = q_MO.x();
            transformStamped_mo.transform.rotation.y = q_MO.y();
            transformStamped_mo.transform.rotation.z = q_MO.z();
            transformStamped_mo.transform.rotation.w = q_MO.w();

            transformStamped_green.header.stamp = ros::Time::now();
            transformStamped_green.transform.translation.x = new_vect_OB.x;
            transformStamped_green.transform.translation.y = new_vect_OB.y;
            transformStamped_green.transform.rotation.x = q_OB.x();
            transformStamped_green.transform.rotation.y = q_OB.y();
            transformStamped_green.transform.rotation.z = q_OB.z();
            transformStamped_green.transform.rotation.w = q_OB.w();

            odom_pub.publish(odom);
            broadcaster.sendTransform(transformStamped);
            broadcaster.sendTransform(transformStamped_mo);
            broadcaster.sendTransform(transformStamped_green);
        }
        
        /// \brief A timer that continuosly publishes paths at 5hz to prevent LAG.
        ///
        void green_loop(const ros::TimerEvent &)
        {
            make_path();
            create_green_path();
        }

    private:
    //create private variables
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub;
    ros::Subscriber obstacles_sub;
    ros::Publisher odom_pub;
    ros::Publisher path_pub;
    ros::Publisher green_path_pub;
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
    geometry_msgs::TransformStamped transformStamped_green;
    geometry_msgs::TransformStamped transformStamped_mo;

    ros::Timer timer;
    ros::Timer timer_green;
    std::vector<double> robot_coords;

    double rate;
    tf2::Quaternion q;
    tf2::Quaternion q_MO;
    tf2::Quaternion q_OB;
    tf2::Quaternion q_MB;
    nav_msgs::Path current_path;
    geometry_msgs::PoseStamped path_pose;

    nav_msgs::Path green_current_path;
    geometry_msgs::PoseStamped green_path_pose;

    int val;
    arma::mat z_values;
    nuslam::KalmanFilter EKFilter;
    arma::mat state;
    int initial_flag;
    int EKF_FLAG;

    turtlelib::Transform2D T_OB;
    turtlelib::Transform2D T_MB;
    turtlelib::Transform2D T_MO;
    
    turtlelib::Vector2D new_vect_MB;
    double new_theta_MB;
    uint32_t shape;
    visualization_msgs::MarkerArray fake_marker;
    ros::Publisher fake_marker_pub;
    double radius;
    double max_range;

    turtlelib::Vector2D current_land;
    std::vector<turtlelib::Vector2D> known_landmarks;
    int landmark_counter;
    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "SLAM");
    SLAM node;
    ros::spin();
    return 0;
}