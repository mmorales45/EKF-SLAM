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
// #include <turtlesim/Pose.h>


class Sim
{
    public:
        Sim() {
            data_val = 0;
            timestep.data = 0;
            nh.getParam("/nusim/rate",rate);
            nh.getParam("/nusim/x0",x0);
            nh.getParam("/nusim/y0",y0);
            nh.getParam("/nusim/theta0",theta0);
            pub = nh.advertise<std_msgs::UInt64>("/nusim/timestep", 1000);
            reset_service = nh.advertiseService("nusim/reset", &Sim::reset, this);
            teleport_service = nh.advertiseService("nusim/teleport", &Sim::teleport, this);
            pub_red_js = nh.advertise<sensor_msgs::JointState>("/red/joint_states", 1000);      
            timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);

            joint_state.name.push_back("red:wheel_left_joint");
            joint_state.name.push_back("red:wheel_right_joint");
            joint_state.position.push_back(0.0);
            joint_state.position.push_back(0.0);

            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "red:base_footprint";
            current_Pose.position.x = x0;
            current_Pose.position.y = y0;
            theta = 0.0;
        }
           
        bool reset(std_srvs::Empty::Request& data, std_srvs::Empty::Response& response)
        {
            timestep.data = 0;
            pub.publish(timestep);
            current_Pose.position.x = 0.0;
            current_Pose.position.y = 0.0;
        return true;
        }

        bool teleport(nusim::Teleport::Request& data, nusim::Teleport::Response& response)
        {
            current_Pose.position.x = data.x;
            current_Pose.position.y = data.y;
            theta = data.theta;
            return true;
        }

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
            q.setRPY(0, 0, theta);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            broadcaster.sendTransform(transformStamped);
         }


    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher pub_red_js;
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
        // nusim::Teleport new_Pose;


};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    Sim node;
    ros::spin();
    return 0;
}