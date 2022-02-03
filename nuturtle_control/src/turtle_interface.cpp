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


#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>


class turtle_interface
{
    public:
        turtle_interface() {
            diffDrive = turtlelib::DiffDrive();

            nh.getParam("/wheel_radius",wheel_radius);
            nh.getParam("/track_width",track_width);
            nh.getParam("/collision_radius",collision_radius);
            // nh.getParam("/motor_cmd_max",motor_cmd_max);
            if(!nh.getParam("/motor_cmd_to_radsec",motor_cmd_to_radsec)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for motor_cmd_to_radsec");
                ros::shutdown();
            }
            else{
                nh.getParam("/motor_cmd_to_radsec",motor_cmd_to_radsec);
            }
            if(!nh.getParam("/encoder_ticks_to_rad",encoder_ticks_to_rad)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for encoder_ticks_to_rad");
                ros::shutdown();
            }
            else{
                nh.getParam("/encoder_ticks_to_rad",encoder_ticks_to_rad);
            }
            if(!nh.getParam("/motor_cmd_max",motor_cmd_max)){
                ROS_INFO_STREAM("Please make sure the parameters are correct! for motor_cmd_max");
                ros::shutdown();
            }
            else{
                nh.getParam("/motor_cmd_max",motor_cmd_max);
            }

            cmd_sub = nh.subscribe("cmd", 1000, &turtle_interface::cmd_callback, this);
            sensor_data_sub = nh.subscribe("sensor_data",1000,&turtle_interface::sensor_data_callback,this);
            
            wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd",1);
            joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
            
        }

        void cmd_callback(const geometry_msgs::Twist & data)
        {
            //data is a twist
            
            
            input_twist.theta_dot = data.angular.z;
            input_twist.x_dot = data.linear.x;
            input_twist.y_dot = data.linear.y;
            wheel_vels = diffDrive.inverse_Kinematics(input_twist);
            wheel_commands.left_velocity = wheel_vels.left_vel;
            wheel_commands.right_velocity = wheel_vels.right_vel;

            wheel_cmd_pub.publish(wheel_commands);
        }

        void sensor_data_callback(const nuturtlebot_msgs::SensorData & sd) 
        {
            wheel_angles.left_angle = sd.left_encoder * encoder_ticks_to_rad;
            wheel_angles.right_angle = sd.right_encoder * encoder_ticks_to_rad;

            wheel_velocitys.left_vel = sd.left_encoder * motor_cmd_to_radsec;
            wheel_velocitys.right_vel = sd.right_encoder * motor_cmd_to_radsec;

            
            wheel_angle_vector.push_back(wheel_angles.left_angle);
            wheel_angle_vector.push_back(wheel_angles.right_angle);
            wheel_velocity_vector.push_back(wheel_velocitys.left_vel);
            wheel_velocity_vector.push_back(wheel_velocitys.right_vel);
            
            jointStates.position = wheel_angle_vector;
            jointStates.velocity = wheel_velocity_vector;

            joint_states_pub.publish(jointStates);

        }
          
    private:
    ros::NodeHandle nh;
    double wheel_radius;
    double track_width;
    double collision_radius;
    double motor_cmd_to_radsec;
    double encoder_ticks_to_rad;
    std::vector<double> motor_cmd_max;

    ros::Subscriber cmd_sub;
    ros::Subscriber sensor_data_sub;
    ros::Publisher wheel_cmd_pub;
    ros::Publisher joint_states_pub;

    turtlelib::speed wheel_velocity;
    // turtlelib::phi_angles wheel_angles;
    turtlelib::DiffDrive diffDrive;
    std::vector<double> wheel_angle_vector;
    std::vector<double> wheel_velocity_vector;
    
    turtlelib::Twist2D input_twist;
    nuturtlebot_msgs::WheelCommands wheel_commands;
    turtlelib::speed wheel_vels;

    turtlelib::phi_angles wheel_angles;
    turtlelib::speed wheel_velocitys;
    sensor_msgs::JointState jointStates;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    turtle_interface node;
    ros::spin();
    return 0;
}