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
///     marker_pub (visualization_msgs/MarkerArray): Creates obstacles as red cylinders in RVIZ
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
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cstdlib>

#include <nuturtlebot_msgs/WheelCommands.h>


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
            nh.getParam("/nusim/x_length",x_length);
            nh.getParam("/nusim/y_length",y_length);
            //Initialize the timer, services, and publishers
            timestep_pub = nh.advertise<std_msgs::UInt64>("/nusim/timestep", 1000);
            marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nusim/obstacles/markerArray", 1, true);
            walls_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nusim/walls/walls", 1, true);
            wheel_cmd_sub = nh.subscribe("red/wheel_cmd", 1000, &Sim::wheel_callback, this);
            
            reset_service = nh.advertiseService("nusim/reset", &Sim::reset, this);
            teleport_service = nh.advertiseService("nusim/teleport", &Sim::teleport, this);
            // joint_state_pub = nh.advertise<sensor_msgs::JointState>("/red/joint_states", 1000);   //remove    
            make_arena();
            timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);
            //Add initial values of 0.0 for joint's position
            joint_state.name.push_back("red-wheel_left_joint");
            joint_state.name.push_back("red-wheel_right_joint");
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(0.0);

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

        void make_arena()
        {
            x_walls.markers.resize(4);
            for (int i = 0;i<4;i++)
            {
                if (i ==0)
                {
                    x_walls.markers[i].pose.position.x = 0;
                    x_walls.markers[i].pose.position.y = -y_length/2 ;
                    x_walls.markers[i].scale.x = x_length +0.1;
                    x_walls.markers[i].scale.y = 0.1;
    
                }
                if (i ==1)
                {
                    x_walls.markers[i].pose.position.x = x_length/2;
                    x_walls.markers[i].pose.position.y = 0;
                    x_walls.markers[i].scale.x = 0.1;
                    x_walls.markers[i].scale.y = y_length+0.1;
    
                }
                if (i ==2)
                {
                    x_walls.markers[i].pose.position.x = 0;
                    x_walls.markers[i].pose.position.y = y_length/2;
                    x_walls.markers[i].scale.x = x_length+0.1;
                    x_walls.markers[i].scale.y = 0.1;
    
                }
                if (i ==3)
                {
                    x_walls.markers[i].pose.position.x = -x_length/2;
                    x_walls.markers[i].pose.position.y = 0;
                    x_walls.markers[i].scale.x = 0.1;
                    x_walls.markers[i].scale.y = y_length+0.1;
    
                }
                x_walls.markers[i].header.stamp = ros::Time();
                x_walls.markers[i].header.frame_id = "world";
                shape = visualization_msgs::Marker::CUBE;
                x_walls.markers[i].type = shape;
                x_walls.markers[i].ns = "walls";
                x_walls.markers[i].action = visualization_msgs::Marker::ADD;
                x_walls.markers[i].id = i;

                x_walls.markers[i].pose.position.z = 0.125;

                x_walls.markers[i].pose.orientation.x = 0.0;
                x_walls.markers[i].pose.orientation.y = 0.0;
                x_walls.markers[i].pose.orientation.z = 0.0;
                x_walls.markers[i].pose.orientation.w = 1.0;

    
                x_walls.markers[i].scale.z = 0.25;
                
                x_walls.markers[i].color.r = 0.13;
                x_walls.markers[i].color.g = 0.54;
                x_walls.markers[i].color.b = 0.13;
                x_walls.markers[i].color.a = 1.0;
            }
            walls_pub.publish(x_walls);

         
        }

        void wheel_callback(const nuturtlebot_msgs::WheelCommands &wheel_commands) 
        {
            left_vel = wheel_commands.left_velocity;
            right_vel = wheel_commands.right_velocity;

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
            // joint_state_pub.publish(joint_state); ///remove
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
            //Create marker array
            marker.markers.resize(cylinders_x_coord.size());
            for (int i = 0;i<(cylinders_x_coord.size());i++)
            {
                marker.markers[i].header.stamp = ros::Time();
                marker.markers[i].header.frame_id = "world";
                shape = visualization_msgs::Marker::CYLINDER;
                marker.markers[i].type = shape;
                marker.markers[i].ns = "obstacles";
                marker.markers[i].action = visualization_msgs::Marker::ADD;
                marker.markers[i].id = i;

                marker.markers[i].pose.position.x = cylinder_marker_x[i];
                marker.markers[i].pose.position.y = cylinder_marker_y[i];
                marker.markers[i].pose.position.z = 0.125;
                marker.markers[i].pose.orientation.x = 0.0;
                marker.markers[i].pose.orientation.y = 0.0;
                marker.markers[i].pose.orientation.z = 0.0;
                marker.markers[i].pose.orientation.w = 1.0;

                marker.markers[i].scale.x = (2*radius);
                marker.markers[i].scale.y = (2*radius);
                marker.markers[i].scale.z = 0.25;
                
                marker.markers[i].color.r = 1.0;
                marker.markers[i].color.g = 0.0;
                marker.markers[i].color.b = 0.0;
                marker.markers[i].color.a = 1.0;
            }
            marker_pub.publish(marker);
         }

    
    private:
        ros::NodeHandle nh;
        ros::Publisher timestep_pub;
        // ros::Publisher joint_state_pub; ///remove
        ros::Publisher marker_pub;
        ros::Publisher walls_pub;
        ros::Subscriber wheel_cmd_sub;

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
        visualization_msgs::MarkerArray marker;
        visualization_msgs::MarkerArray x_walls;

        uint32_t shape;
        int num_obstacles;
        std::vector<double> cylinders_x_coord;
        std::vector<double> cylinders_y_coord;
        std::vector<double> cylinder_marker_x;
        std::vector<double> cylinder_marker_y;
        std::vector<double> robot_coords;
        double radius;
        double x_length;
        double y_length;
        
        int left_vel;
        int right_vel;

};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    Sim node;
    ros::spin();
    return 0;
}