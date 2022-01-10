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

// #include <turtlesim/Pose.h>


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

            pub = nh.advertise<std_msgs::UInt64>("/nusim/timestep", 1000);
            marker_pub  = nh.advertise<visualization_msgs::Marker>("/obstacles/visualization", 1000, true);
            reset_service = nh.advertiseService("nusim/reset", &Sim::reset, this);
            teleport_service = nh.advertiseService("nusim/teleport", &Sim::teleport, this);
            pub_red_js = nh.advertise<sensor_msgs::JointState>("/red/joint_states", 1000);      
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
            theta = 0.0;

        }
           
        
        double fRand(double fMin, double fMax)
        {
            double f = (double)rand() / RAND_MAX;
            return fMin + f * (fMax - fMin);
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
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
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
        ros::Publisher pub;
        ros::Publisher pub_red_js;
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
        // nusim::Teleport new_Pose;
        double randNumx;
        double randNumy;
        std::vector<double> rand_x;
        std::vector<double> rand_y;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nodename");
    Sim node;
    ros::spin();
    return 0;
}