/// \file
/// \brief Node that detects landmarks
///
/// PARAMETERS:
///     
/// PUBLISHERS:
///     
/// SUBSCRIBERS:
///     
/// SERVICES:
///     


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

#include <sensor_msgs/LaserScan.h>


/// \brief Create and update the odometry between the world and blue turtlebot

class Landmarks
{
    public:
        Landmarks() {
            
            laser_sub = nh.subscribe("/nusim/laser_scan_data",10,&Landmarks::ls_callback,this);
            timer = nh.createTimer(ros::Duration(1.0/500.0), &Landmarks::main_loop, this);
            cluster_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/points/Clusters", 1, true);

            cluster_thresh = 0.1;
            min_laserScan_range = 0.2;
        }

        
        void showCluster(std::vector<std::vector<turtlelib::Vector2D>> main_cluster)
        {
            int counter = 0;
            int id_counter = 0;
            int size_main = main_cluster.size();
            // visualization_msgs::MarkerArray cluster_marker;
            for (int i = 0;i<size_main;i++)
            {
                int size_cluster = main_cluster[i].size();
                for (int j = 0;j< size_cluster;j++)
                {
                    counter++;
                }
            }
            cluster_marker.markers.resize(counter);
            for (int i = 0;i<size_main;i++)
            {
                int size_cluster = main_cluster[i].size();
                std::vector<turtlelib::Vector2D> currentCluster = main_cluster[i];
                for (int j = 0;j< size_cluster;j++)
                {
                    cluster_marker.markers[id_counter].header.stamp = ros::Time();
                    cluster_marker.markers[id_counter].header.frame_id = "red-base_footprint";
                    uint32_t shape;
                    shape = visualization_msgs::Marker::SPHERE;
                    cluster_marker.markers[id_counter].type = shape;
                    cluster_marker.markers[id_counter].ns = "foundPoints";
                    cluster_marker.markers[id_counter].action = visualization_msgs::Marker::ADD;
                    cluster_marker.markers[id_counter].id = id_counter;

                    cluster_marker.markers[id_counter].pose.position.x = currentCluster[j].x;
                    cluster_marker.markers[id_counter].pose.position.y = currentCluster[j].y;
                    cluster_marker.markers[id_counter].pose.position.z = 0.2;
                    cluster_marker.markers[id_counter].pose.orientation.x = 0.0;
                    cluster_marker.markers[id_counter].pose.orientation.y = 0.0;
                    cluster_marker.markers[id_counter].pose.orientation.z = 0.0;
                    cluster_marker.markers[id_counter].pose.orientation.w = 1.0;

                    cluster_marker.markers[id_counter].scale.x = (0.05);
                    cluster_marker.markers[id_counter].scale.y = (0.05);
                    cluster_marker.markers[id_counter].scale.z = (0.05);
                    
                    cluster_marker.markers[id_counter].color.r = 1.0;
                    cluster_marker.markers[id_counter].color.g = 1.0;
                    cluster_marker.markers[id_counter].color.b = 0.0;
                    cluster_marker.markers[id_counter].color.a = 1.0;
                    // cluster_marker.markers[id_counter].frame_locked = true;
                    id_counter++;
                }

            }
            cluster_marker_pub.publish(cluster_marker); 
        }

        /// \brief Callback for the fake sensor data topic.
        ///
        void ls_callback(const sensor_msgs::LaserScan & laserData)
        {
            //clear the main cluster every run
            std::vector<std::vector<turtlelib::Vector2D>> main_cluster;
            //get the size of the laser scan array
            int size_laser = laserData.ranges.size();
            std::vector<turtlelib::Vector2D> cluster;
            for (int i = 1; i< size_laser; i++)
            {
                double old_range, new_range;
                old_range = laserData.ranges.at(i-1);
                new_range = laserData.ranges.at(i);
                
                // if the new range is smaller than the laser min, check cluster size, if any, then break out of current angle.
                if ( (new_range <= min_laserScan_range) && ((i+1)!=size_laser) )
                {
                    if (cluster.size()>=3){
                        main_cluster.push_back(cluster);
                        cluster.clear();
                    }
                    continue;
                }

                turtlelib::Vector2D old_xy, new_xy;
                old_xy.x = old_range * cos(turtlelib::deg2rad(i-1));
                old_xy.y = old_range * sin(turtlelib::deg2rad(i-1));
                new_xy.x = new_range * cos(turtlelib::deg2rad(i));
                new_xy.y = new_range * sin(turtlelib::deg2rad(i));;
                //only for the very first cluster and at angle 0
                // if (cluster.size() == 0)
                // {
                //     cluster.push_back(old_xy);
                // }

                if (fabs(new_range-old_range) < cluster_thresh) 
                {
                    cluster.push_back(new_xy);
                }
                //found a point outside of the current cluster
                else
                {
                    if (cluster.size()>=3){
                        main_cluster.push_back(cluster);
                    }
                    cluster.clear();
                    if ((i+1) == size_laser){
                        cluster.push_back(new_xy);
                    }
                }


                //check for the scenario where the first cluster is in front of the robot
                if ((i+1) == size_laser)
                {
                    double init_range = laserData.ranges.at(0);
                    turtlelib::Vector2D init_xy = main_cluster[0][0];
                    double delta_x,delta_y;
                    delta_x = (init_range*cos(0))-init_xy.x;
                    delta_y = (init_range*sin(0))-init_xy.y;

                    if ( turtlelib::almost_equal(delta_x,0.0,0.01) && turtlelib::almost_equal(delta_y,0.0,0.01))
                    {
                        if (fabs(new_range-init_range) < cluster_thresh)
                        {
                            main_cluster[0].insert(main_cluster[0].begin(),cluster.begin(),cluster.end());
                            cluster.clear();
                        }
                    }
                    if (cluster.size()>=3)
                    {
                        main_cluster.push_back(cluster);
                    }

                    if (main_cluster[0].size() < 3)
                    {
                        main_cluster.erase(main_cluster.begin());
                    }
                }

            }
            showCluster(main_cluster);
        }

       
        /// \brief A timer that continuosly publishes odometry, creates transforms, and updates the DiffDrive class members
        ///
        void main_loop(const ros::TimerEvent &)
        {
            
        }

    private:
    //create private variables
    ros::NodeHandle nh;
    ros::Subscriber laser_sub;
    ros::Publisher cluster_marker_pub;
    ros::Timer timer;
    
    //create variable for cluster distance
    double cluster_thresh;

    double min_laserScan_range;
    visualization_msgs::MarkerArray cluster_marker;
    
    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "LANDMARKS");
    Landmarks node;
    ros::spin();
    return 0;
}