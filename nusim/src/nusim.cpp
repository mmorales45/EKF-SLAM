/// \file
/// \brief This file creates custom services that affect the turtlebot as well as create obstacles in RVIZ.
///
/// PARAMETERS:
///     rate (double): Frequency at which the main loop runs
///     cylinders_x_coord (std::vector<double>): The x coordaintes of the three red cylinders
///     cylinders_y_coord (std::vector<double>): The y coordaintes of the three red cylinders
///     robot (std::vector<double>): vector/array of the robot's iniital coordinates in x,y,and theta
///     radius (double): the radius of the red cylinders
/// PUBLISHES:
///     timestep_pub (std_msgs/UInt64): Tracks the simulation's current timestep
///     joint_state_pub (sensor_msgs/JointState): Publishes joint positions to red/joint_states
///     marker_pub (visualization_msgs/Marker): Creates obstacles as red cylinders in RVIZ
/// SERVICES:
///     reset (nusim/reset): Sets the timestep to 0 and teleports turtlebot to the initial position
///     teleport (nusim/Teleport): Teleports the turtblebot to a user defined position

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
#include <nusim/Teleport.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cstdlib>

/// \brief Creates a simulator and visualizer for the turtlebot3

class Sim
{
    public:
        Sim() {
            //get parameters from basic_world.yaml
            timestep.data = 0;
            nh.getParam("/nusim/rate",rate);
            nh.getParam("/nusim/cylinders_x_coord",cylinders_x_coord);
            nh.getParam("/nusim/cylinders_y_coord",cylinders_y_coord);
            nh.getParam("/nusim/robot",robot_coords);
            nh.getParam("/nusim/radius",radius);
            //Initialize the timer, services, and publishers
            timestep_pub = nh.advertise<std_msgs::UInt64>("/nusim/timestep", 1000);
            marker_pub  = nh.advertise<visualization_msgs::Marker>("/nusim/obstacles/visualization", 1000, true);
            reset_service = nh.advertiseService("nusim/reset", &Sim::reset, this);
            teleport_service = nh.advertiseService("nusim/teleport", &Sim::teleport, this);
            joint_state_pub = nh.advertise<sensor_msgs::JointState>("/red/joint_states", 1000);      
            timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);
            //Add initial values of 0.0 for joint's position
            joint_state.name.push_back("red-wheel_left_joint");
            joint_state.name.push_back("red-wheel_right_joint");
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(0.0);
            //initialize parameters of marker that do not change such as the name,type and ns
            marker.header.frame_id = "world";
            marker.ns = "/nusim/obstacles/obstacles";
            shape = visualization_msgs::Marker::CYLINDER;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
            //set initial coordinates for the robot as well as the header and child ids
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "red-base_footprint";
            current_Pose.position.x = robot_coords[0];
            current_Pose.position.y = robot_coords[1];
            theta = robot_coords[2];
            q.setRPY(0, 0, theta);
            //add values to x and y coordiante vectors
            for (int j = 0;j<(cylinders_x_coord.size());j++){
                cylinder_marker_x.push_back(cylinders_x_coord[j]);
                cylinder_marker_y.push_back(cylinders_y_coord[j]);
            }

        }
    
        /// \brief sets the timestep to 0 and teleports robot to intial position
        ///
        /// \param data - empty
        /// \returns response - true 
        bool reset(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {
            timestep.data = 0;
            timestep_pub.publish(timestep);
            current_Pose.position.x = robot_coords[0];
            current_Pose.position.y = robot_coords[1];
            theta = robot_coords[2];
        return true;
        }

        /// \brief teleports the robot to a position set by the user
        ///
        /// \param data - Teleport data type composed of user x,y,theta
        /// \returns response - true 
        bool teleport(nusim::Teleport::Request& data, nusim::Teleport::Response& response)
        {
            current_Pose.position.x = data.x;
            current_Pose.position.y = data.y;
            theta = data.theta;
            return true;
        }

        /// \brief A timer that updates the simulation
        ///
        void main_loop(const ros::TimerEvent &)
         {
            joint_state.header.stamp = ros::Time::now();
            joint_state.position[0] = 0.0;
            joint_state.position[1] = 0.0;
            //update timestep
            timestep.data++;
            timestep_pub.publish(timestep);  
            joint_state_pub.publish(joint_state);
            //update position of the robot 
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = current_Pose.position.x;
            transformStamped.transform.translation.y = current_Pose.position.y;
            transformStamped.transform.translation.z = current_Pose.position.z;
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            broadcaster.sendTransform(transformStamped);
            //publish the markers'/cylinders' position 
            for (int i = 0;i<(cylinders_x_coord.size());i++)
            {
                marker.id = i;
                marker.pose.position.x = cylinder_marker_x[i];
                marker.pose.position.y = cylinder_marker_y[i];
                marker.pose.position.z = 0.125;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = (2*radius);
                marker.scale.y = (2*radius);
                marker.scale.z = 0.25;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
                marker.lifetime = ros::Duration();
                marker_pub.publish(marker);
            }
         }

    
    private:
        ros::NodeHandle nh;
        ros::Publisher timestep_pub;
        ros::Publisher joint_state_pub;
        ros::Publisher marker_pub;
        ros::Timer timer;
        double rate;
        std_msgs::UInt64 timestep;
        ros::ServiceServer reset_service;
        ros::ServiceServer teleport_service;
        sensor_msgs::JointState joint_state;
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;
        tf2::Quaternion q;
        geometry_msgs::Pose current_Pose;
        double theta;
        visualization_msgs::Marker marker;
        uint32_t shape;
        int num_obstacles;
        std::vector<double> cylinders_x_coord;
        std::vector<double> cylinders_y_coord;
        std::vector<double> cylinder_marker_x;
        std::vector<double> cylinder_marker_y;
        std::vector<double> robot_coords;
        double radius;

};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    Sim node;
    ros::spin();
    return 0;
}