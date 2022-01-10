/// \file
/// \brief This file creates custom services that affect the turtlebot as well as create obstacles in RVIZ.
///
/// PARAMETERS:
///     rate (double): Frequency at which the main loop runs
///     parameter_name (parameter_type): description of the parameter
///     parameter_name (parameter_type): description of the parameter
///     parameter_name (parameter_type): description of the parameter
/// PUBLISHES:
///     timestep_pub (std_msgs/UInt64): Tracks the simulation's current timestep
///     joint_state_pub (sensor_msgs/JointState): Publishes joint positions to red/joint_states
///     marker_pub (visualization_msgs/Marker): Creates obstacles as red cylinders
/// SERVICES:
///     reset (std_srvs/Empty): Sets the timestep to 0 and teleports turtlebot to (0,0,0)
///     teleport (nusim/Teleport): Teleports the turtblebot to a user defined position
///     service_name (service_type): description of the service
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


/// \brief Computes the factorial
///
/// \tparam T - An integral type
/// \param F - the factorial to compute
/// \returns F! (F Factorial)
/// <template typename T>
/// T factorial (T F);
class Sim
{
    public:
        Sim() {
            srand ( (unsigned)time(NULL));

            data_val = 0;
            timestep.data = 0;
            nh.getParam("/nusim/rate",rate);
            nh.getParam("/nusim/x0",x0);
            nh.getParam("/nusim/y0",y0);
            nh.getParam("/nusim/theta0",theta0);
            nh.getParam("/nusim/num_obstacles",num_obstacles);
            nh.getParam("/cylinders_x_coord",cylinders_x_coord);
            nh.getParam("/cylinders_y_coord",cylinders_y_coord);
            nh.getParam("/robot",robot_coords);
            nh.getParam("/radius",radius);
            radius = radius*2.0;

            for (int j = 0;j<(cylinders_x_coord.size()+num_obstacles);j++){
                randNumx = fRand(-2.5,2.5);
                randNumy = fRand(-2.5,2.5);

                if (j<cylinders_x_coord.size()) {
                    cylinder_marker_x.push_back(cylinders_x_coord[j]);
                    cylinder_marker_y.push_back(cylinders_y_coord[j]);
                    std::cout << cylinder_marker_x[j] << " X\n";
                    std::cout << cylinder_marker_y[j] << " Y\n";
                }
                else {
                    cylinder_marker_x.push_back(randNumx);
                    cylinder_marker_y.push_back(randNumy);
                }
            }

            timestep_pub = nh.advertise<std_msgs::UInt64>("/nusim/timestep", 1000);
            marker_pub  = nh.advertise<visualization_msgs::Marker>("/obstacles/visualization", 1000, true);
            reset_service = nh.advertiseService("nusim/reset", &Sim::reset, this);
            teleport_service = nh.advertiseService("nusim/teleport", &Sim::teleport, this);
            joint_state_pub = nh.advertise<sensor_msgs::JointState>("/red/joint_states", 1000);      
            timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);

            joint_state.name.push_back("red:wheel_left_joint");
            joint_state.name.push_back("red:wheel_right_joint");
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(0.0);

            marker.header.frame_id = "world";
            marker.ns = "/obstacles/obstacles";
            shape = visualization_msgs::Marker::CYLINDER;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
        
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "red:base_footprint";
            current_Pose.position.x = x0;
            current_Pose.position.y = y0;
            q.setRPY(0, 0, theta);
            // current_Pose.position.x = robot_coords[0];
            // current_Pose.position.y = robot_coords[1];
            theta = 0.0;
            // q.setRPY(0, 0, robot_coords[2]);

        }
           
        /// \brief Computes the factorial
        ///
        /// \tparam T - An integral type
        /// \param F - the factorial to compute
        /// \returns F! (F Factorial)
        /// <template typename T>
        /// T factorial (T F);
        double fRand(double fMin, double fMax)
        {
            double f = (double)rand() / RAND_MAX;
            return fMin + f * (fMax - fMin);
        }

        /// \brief Computes the factorial
        ///
        /// \tparam T - An integral type
        /// \param F - the factorial to compute
        /// \returns F! (F Factorial)
        /// <template typename T>
        /// T factorial (T F);
        bool reset(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {
            timestep.data = 0;
            pub.publish(timestep);
            current_Pose.position.x = 0.0;
            current_Pose.position.y = 0.0;
        return true;
        }

        /// \brief Computes the factorial
        ///
        /// \tparam T - An integral type
        /// \param F - the factorial to compute
        /// \returns F! (F Factorial)
        /// <template typename T>
        /// T factorial (T F);
        bool teleport(nusim::Teleport::Request& data, nusim::Teleport::Response& response)
        {
            current_Pose.position.x = data.x;
            current_Pose.position.y = data.y;
            theta = data.theta;
            return true;
        }

        /// \brief Computes the factorial
        ///
        /// \tparam T - An integral type
        /// \param F - the factorial to compute
        /// \returns F! (F Factorial)
        /// <template typename T>
        /// T factorial (T F);
        void main_loop(const ros::TimerEvent &)
         {
            // implement the state machine here
            joint_state.header.stamp = ros::Time::now();
            joint_state.position[0] = 0.0;
            joint_state.position[1] = 0.0;

            timestep.data++;
            pub.publish(timestep);  
            pub_red_js.publish(joint_state);

            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = current_Pose.position.x;
            transformStamped.transform.translation.y = current_Pose.position.y;
            transformStamped.transform.translation.z = current_Pose.position.z;
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            broadcaster.sendTransform(transformStamped);

            for (int i = 0;i<(num_obstacles+cylinders_x_coord.size());i++)
            {
                marker.id = i;
                marker.pose.position.x = cylinder_marker_x[i];
                marker.pose.position.y = cylinder_marker_y[i];
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = radius;
                marker.scale.y = radius;
                marker.scale.z = 0.25;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
                marker.lifetime = ros::Duration();
                marker_pub.publish(marker);
            }
                // if (i<3) {

                // }
            


         }


    private:
        ros::NodeHandle nh;
        ros::Publisher timestep_pub;
        ros::Publisher joint_state_pub;
        ros::Publisher marker_pub;
        ros::Timer timer;
        double rate;
        std_msgs::UInt64 timestep;
        int data_val;
        ros::ServiceServer reset_service;
        ros::ServiceServer teleport_service;
        sensor_msgs::JointState joint_state;
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;
        tf2::Quaternion q;
        geometry_msgs::Pose current_Pose;
        double x0;
        double y0;
        double theta0;
        double theta;
        visualization_msgs::Marker marker;
        uint32_t shape;
        int num_obstacles;
        std::vector<double> cylinders_x_coord;
        std::vector<double> cylinders_y_coord;
        std::vector<double> cylinder_marker_x;
        std::vector<double> cylinder_marker_y;
        std::vector<double> robot_coords;
        // nusim::Teleport new_Pose;
        double randNumx;
        double randNumy;
        std::vector<double> rand_x;
        std::vector<double> rand_y;
        double radius;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    Sim node;
    ros::spin();
    return 0;
}