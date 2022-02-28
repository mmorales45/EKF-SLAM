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
            odom.header.frame_id = "odom";
            odom.child_frame_id = "green-base_footprint";
            transformStamped.header.frame_id = "world";// for blue robot
            transformStamped.child_frame_id = "blue-base_footprint";// for blue robot
            transformStamped_mo.header.frame_id = "map";// for green robot
            transformStamped_mo.child_frame_id = "odom";// for green robot
            transformStamped_green.header.frame_id = "odom";// for green robot
            transformStamped_green.child_frame_id = "green-base_footprint";// for green robot
            nh.getParam("/robot",robot_coords);
            current_config.x = robot_coords[0];
            current_config.y = robot_coords[1];
            current_config.theta = robot_coords[2];
            DiffDrive = turtlelib::DiffDrive(current_config);
            
            joint_state_sub = nh.subscribe("/joint_states",10,&odometry::js_callback,this);
            obstacles_sub = nh.subscribe("/nusim/obstacles/Fake_markerArray",10,&odometry::obstacle_callback,this);
            odom_pub = nh.advertise<nav_msgs::Odometry>("odom",100);
            path_pub  = nh.advertise<nav_msgs::Path>("odometry/path", 10, true);
            green_path_pub = nh.advertise<nav_msgs::Path>("odometry/green_path", 10, true);
            set_pose_service = nh.advertiseService("set_pose", &odometry::set_pose_callback, this);
            fake_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nuslam/obstacles/Fake_markerArray", 1, true);

            EKFilter = nuslam::KalmanFilter();
            c = arma::mat(3*(3),1,arma::fill::zeros);
            initial_flag = 0;
            b = arma::mat(3*(3),1,arma::fill::zeros);
            timer = nh.createTimer(ros::Duration(1.0/rate), &odometry::main_loop, this);
            timer_green = nh.createTimer(ros::Duration(1.0/5.0), &odometry::green_loop, this);
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
            val = int (fake_obstacles.markers.size());
            z_values = arma::mat(2*(val),1,arma::fill::zeros);
            
            for (int t = 0;t<val;t++)
            {
                double x_i = fake_obstacles.markers[t].pose.position.x;
                double y_i = fake_obstacles.markers[t].pose.position.y;
                turtlelib::Vector2D coord_vect;
                
                double radius_i = sqrt(pow(x_i,2)+pow(y_i,2));
                double phi_i = turtlelib::normalize_angle(atan2(y_i,x_i));
                coord_vect.x = radius_i;
                coord_vect.y = phi_i;
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

            c = EKFilter.predict(twist,5.0);
            // ROS_INFO_STREAM(c);

            b = EKFilter.update(val,z_values); 
            ROS_INFO_STREAM(b);

            make_fake_obstacles();

            
        }
        
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

        void make_fake_obstacles()
        {            
            fake_marker.markers.resize(val);
            for (int i = 0;i<val;i++)
            {
                turtlelib::Vector2D obstacle_B_MARK;
                obstacle_B_MARK.x = b(3+2*i,0);
                obstacle_B_MARK.y = b(4+2*i,0);


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

                fake_marker.markers[i].scale.x = (2*0.05);
                fake_marker.markers[i].scale.y = (2*0.05);
                fake_marker.markers[i].scale.z = 0.25;
                
                fake_marker.markers[i].color.r = 0.0;
                fake_marker.markers[i].color.g = 1.0;
                fake_marker.markers[i].color.b = 0.0;
                fake_marker.markers[i].color.a = 1.0;

            }
            fake_marker_pub.publish(fake_marker);
            
        }


        /// \brief A timer that continuosly publishes odometry, creates transforms, and updates the DiffDrive class members
        ///
        void main_loop(const ros::TimerEvent &)
        {
            make_path();
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
            vect_MB.x = b(1,0);
            vect_MB.y = b(2,0);
            double theta_MB = b(0,0);
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
            odom.pose.pose.position.x = current_config.x;
            odom.pose.pose.position.y = current_config.y;
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
            q_OB.setRPY(0, 0, new_theta_OB);
            transformStamped_green.transform.rotation.x = q_OB.x();
            transformStamped_green.transform.rotation.y = q_OB.y();
            transformStamped_green.transform.rotation.z = q_OB.z();
            transformStamped_green.transform.rotation.w = q_OB.w();

            odom_pub.publish(odom);
            broadcaster.sendTransform(transformStamped);
            broadcaster.sendTransform(transformStamped_mo);
            broadcaster.sendTransform(transformStamped_green);
        }
        
        void green_loop(const ros::TimerEvent &)
        {
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
    arma::mat c;
    arma::mat b;
    int initial_flag;

    turtlelib::Transform2D T_OB;
    turtlelib::Transform2D T_MB;
    turtlelib::Transform2D T_MO;
    
    turtlelib::Vector2D new_vect_MB;
    double new_theta_MB;
    uint32_t shape;
    visualization_msgs::MarkerArray fake_marker;
    ros::Publisher fake_marker_pub;
    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry");
    odometry node;
    ros::spin();
    return 0;
}