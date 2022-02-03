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


#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>
#include <nav_msgs/Odometry.h>
#include <nuturtle_control/Configuration.h>

class odometry
{
    public:
        odometry() {
            DiffDrive = turtlelib::DiffDrive();
            loadParams();
            joint_state_sub = nh.subscribe("joint_states",1000,&odometry::js_callback,this);
            odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1);
            set_pose_service = nh.advertiseService("set_pose", &odometry::set_pose_callback, this);
        }

        void js_callback(const sensor_msgs::JointState & js)
        {

            new_vel.left_vel = js.velocity[0];
            new_vel.right_vel = js.velocity[1];
            twist = DiffDrive.Twist_from_wheelVel(new_vel);

            new_angles.left_angle = js.position[0] - old_angles.left_angle;
            new_angles.right_angle = js.position[1] - old_angles.right_angle;
            current_config = DiffDrive.forward_Kinematics(new_angles);

            old_angles.left_angle = new_angles.left_angle;
            old_angles.right_angle = new_angles.right_angle;

            /////////////////////////////////////////////////////////

            odom.header.frame_id = odom_id;
            odom.header.stamp = ros::Time::now();
            odom.child_frame_id = body_id;
            odom.pose.pose.position.x = current_config.x;
            odom.pose.pose.position.y = current_config.y;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_config.theta);
            odom.pose.pose.orientation = odom_quat;

            odom.twist.twist.angular.z = twist.theta_dot;
            odom.twist.twist.linear.x = twist.x_dot;
            odom.twist.twist.linear.y = twist.y_dot;
            odom_pub.publish(odom);

            /////////////////////////////////////////////
            transformStamped.header.frame_id = odom_id;
            transformStamped.child_frame_id = body_id;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = current_config.x;
            transformStamped.transform.translation.y = current_config.y;
            transformStamped.transform.rotation = odom_quat;

            broadcaster.sendTransform(transformStamped);



        }

        bool set_pose_callback(nuturtle_control::Configuration::Request& data, nuturtle_control::Configuration::Request& )
        {
            current_config.theta = data.theta;
            current_config.x = data.x;
            current_config.y = data.y;
            DiffDrive = turtlelib::DiffDrive(current_config);
            return true;
        }

        void loadParams()
        {
            if(!nh.getParam("/odom_id",odom_id)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for motor_cmd_to_radsec");
                ros::shutdown();
            }
            else{
                nh.getParam("/odom_id",odom_id);
            }
            if(!nh.getParam("/body_id",body_id)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for motor_cmd_to_radsec");
                ros::shutdown();
            }
            else{
                nh.getParam("/body_id",body_id);
            }
            if(!nh.getParam("/left_wheel_joint",left_wheel_joint)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for motor_cmd_to_radsec");
                ros::shutdown();
            }
            else{
                nh.getParam("/left_wheel_joint",left_wheel_joint);
            }
            if(!nh.getParam("/right_wheel_joint",right_wheel_joint)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for motor_cmd_to_radsec");
                ros::shutdown();
            }
            else{
                nh.getParam("/right_wheel_joint",right_wheel_joint);
            }
        }
    private:
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub;
    ros::Publisher odom_pub;
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

    
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry");
    odometry node;
    ros::spin();
    return 0;
}