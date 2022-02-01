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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cstdlib>


#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>


class turtle_interface
{
    public:
        turtle_interface() {
            nh.getParam("/wheel_radius",wheel_radius);
            nh.getParam("/track_width",track_width);
            nh.getParam("/collision_radius",collision_radius);
            nh.getParam("/motor_cmd_to_radsec",motor_cmd_to_radsec);
            nh.getParam("/encoder_ticks_to_rad",encoder_ticks_to_rad);
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

            // cmd_sub = nh.subscribe("cmd", 1000, &turtle_interface::cmd_callback, this);
            // wheel_cmd_pub = ;

            // cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, &turtle_interface::callback, this);
        }

        // void cmd_callback(nuturtlebot_msgs::SensorData & sd)
        // {
            
        // }
          
    private:
    ros::NodeHandle nh;
    double wheel_radius;
    double track_width;
    double collision_radius;
    double motor_cmd_to_radsec;
    double encoder_ticks_to_rad;
    std::vector<double> motor_cmd_max;

    ros::Subscriber cmd_sub;
    ros::Publisher wheel_cmd_pub;

};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_interface");
    turtle_interface node;
    ros::spin();
    return 0;
}