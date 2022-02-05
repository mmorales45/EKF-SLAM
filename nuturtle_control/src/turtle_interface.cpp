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
            rate = 500;
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

            cmd_sub = nh.subscribe("cmd_vel", 10, &turtle_interface::cmd_callback, this);
            sensor_data_sub = nh.subscribe("sensor_data",10,&turtle_interface::sensor_data_callback,this);
            
            wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd",10);
            joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",10);
            
            jointStates.header.stamp = ros::Time::now();
            jointStates.position.push_back(0);
            jointStates.position.push_back(0);
            jointStates.velocity.push_back(0);
            jointStates.velocity.push_back(0);
            
            jointStates.name.push_back("red-wheel_left_joint");
            jointStates.name.push_back("red-wheel_right_joint");
            timer = nh.createTimer(ros::Duration(1/rate), &turtle_interface::main_loop, this);
        }

        void cmd_callback(const geometry_msgs::Twist & data)
        {
            //data is a twist
            input_twist.theta_dot = data.angular.z;
            input_twist.x_dot = data.linear.x;
            input_twist.y_dot = data.linear.y;
            wheel_vels = diffDrive.inverse_Kinematics(input_twist);

            
        }

        void sensor_data_callback(const nuturtlebot_msgs::SensorData & sd) 
        {
            wheel_angles.left_angle = sd.left_encoder * encoder_ticks_to_rad;
            wheel_angles.right_angle = sd.right_encoder * encoder_ticks_to_rad;


            // ROS_WARN("left: %f right: %f",wheel_angles.left_ang21416le,wheel_angles.right_angle);
            wheel_velocitys.left_vel = (sd.left_encoder-old_left)* motor_cmd_to_radsec;
            wheel_velocitys.right_vel = (sd.right_encoder-old_right) * motor_cmd_to_radsec;
            ROS_WARN("left new: %d left old: %d right new: %d right old: %d",sd.left_encoder,old_left,sd.right_encoder,old_right);
            old_left = sd.left_encoder;
            old_right = sd.right_encoder;
            // ROS_WARN("left: %f right: %f",wheel_velocitys.left_vel,wheel_velocitys.right_vel);

            // wheel_cmd_pub.publish(wheel_commands);

        }

        void main_loop(const ros::TimerEvent &)
        {
            // wheel_vels = diffDrive.inverse_Kinematics(input_twist);
            wheel_commands.left_velocity = wheel_vels.left_vel;
            wheel_commands.right_velocity = wheel_vels.right_vel;
            

            jointStates.position[0] = (wheel_angles.left_angle);
            jointStates.position[1] = (wheel_angles.right_angle);
            jointStates.velocity[0] = (wheel_velocitys.left_vel);
            jointStates.velocity[1] = (wheel_velocitys.right_vel);
            // ROS_WARN("left: %f right: %f",jointStates.position[0],jointStates.position[1]);
            // ROS_WARN("left: %f right: %f",jointStates.velocity[0],jointStates.velocity[1]);
            
            // jointStates.position = wheel_angle_vector;
            // jointStates.velocity = wheel_velocity_vector;
            jointStates.header.stamp = ros::Time::now();

            wheel_cmd_pub.publish(wheel_commands);
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
    ros::Timer timer;

    int old_left;
    int old_right;

    double rate;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    turtle_interface node;
    ros::spin();
    return 0;
}