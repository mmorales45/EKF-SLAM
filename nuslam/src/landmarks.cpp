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

#include <nuslam/CircleFitting.hpp>


/// \brief Create and update the odometry between the world and blue turtlebot

class Landmarks
{
    public:
        Landmarks() {
            //initialize the subscribers and publishers
            laser_sub = nh.subscribe("/nusim/laser_scan_data",10,&Landmarks::ls_callback,this);
            timer = nh.createTimer(ros::Duration(1.0/500.0), &Landmarks::main_loop, this);
            cluster_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/points/Clusters", 1, true);
            obstacle_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/points/Obstacles", 1, true);
            //define cluster thresh
            cluster_thresh = 0.05;
            min_laserScan_range = 0.05;
        }

        
        void showObstacles()
        {
            int counter = 0;
            int size_main = R_array.size();
            obstacle_marker.markers.resize(size_main);
            for (int i = 0;i<size_main;i++)
            {
                obstacle_marker.markers[i].header.stamp = ros::Time();
                obstacle_marker.markers[i].header.frame_id = "blue-base_scan";
                uint32_t shape;
                shape = visualization_msgs::Marker::CYLINDER;
                obstacle_marker.markers[i].type = shape;
                obstacle_marker.markers[i].ns = "Obstacles";
                if (R_array.at(i) < 0.1)
                {
                    obstacle_marker.markers[i].action = visualization_msgs::Marker::ADD;
                }
                else
                {
                    obstacle_marker.markers[i].action = visualization_msgs::Marker::DELETE;
                }
                // obstacle_marker.markers[i].action = visualization_msgs::Marker::ADD;
                obstacle_marker.markers[i].id = i;

                obstacle_marker.markers[i].pose.position.x = xy_COORDS.at(i).x;
                obstacle_marker.markers[i].pose.position.y = xy_COORDS.at(i).y;
                obstacle_marker.markers[i].pose.position.z = -0.065;
                obstacle_marker.markers[i].pose.orientation.x = 0.0;
                obstacle_marker.markers[i].pose.orientation.y = 0.0;
                obstacle_marker.markers[i].pose.orientation.z = 0.0;
                obstacle_marker.markers[i].pose.orientation.w = 1.0;

                obstacle_marker.markers[i].scale.x = 2*R_array.at(i);
                obstacle_marker.markers[i].scale.y = 2*R_array.at(i);
                obstacle_marker.markers[i].scale.z = (0.25);
                
                obstacle_marker.markers[i].color.r = 1.0;
                obstacle_marker.markers[i].color.g = 1.0;
                obstacle_marker.markers[i].color.b = 0.0;
                obstacle_marker.markers[i].color.a = 1.0;
            }
            obstacle_marker_pub.publish(obstacle_marker); 
        }

        void showCluster()
        {
            int counter = 0;
            int id_counter = 0;
            int size_main = main_cluster.size();
            // std::cout << "________GOT TO HERE 0____________" << std::endl;
            // visualization_msgs::MarkerArray cluster_marker;
            for (int i = 0;i<size_main;i++)
            {
                int size_cluster = main_cluster[i].size();
                for (int j = 0;j< size_cluster;j++)
                {
                    counter++;
                }
            }
            // std::cout << "________GOT TO HERE 1____________" << std::endl;
            cluster_marker.markers.resize(counter);
            for (int i = 0;i<size_main;i++)
            {
                int size_cluster = main_cluster[i].size();
                std::vector<turtlelib::Vector2D> currentCluster = main_cluster[i];
                for (int j = 0;j< size_cluster;j++)
                {
                    cluster_marker.markers[id_counter].header.stamp = ros::Time();
                    cluster_marker.markers[id_counter].header.frame_id = "blue-base_scan";
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
                // std::cout << "________GOT TO HERE 2___________" << std::endl;

            }
            // std::cout << "________GOT TO HERE 3___________" << std::endl;
            cluster_marker_pub.publish(cluster_marker); 
            // std::cout << "________GOT TO HERE 4___________" << std::endl;
        }

        // check_mainCluster_size();
        /// \brief Callback for the fake sensor data topic.
        ///
        void ls_callback(const sensor_msgs::LaserScan & laserData)
        {
            //clear the main cluster every run
            main_cluster.clear();

            //get the size of the laser scan array
            int size_laser = laserData.ranges.size();
            std::vector<turtlelib::Vector2D> cluster;
            std::vector<double> cluster_array;
            std::vector<std::vector<double>> range_array;

            turtlelib::Vector2D init_XY;
            init_XY = { laserData.ranges.at(0) * cos(turtlelib::deg2rad(0)),
                        laserData.ranges.at(0) * sin(turtlelib::deg2rad(0))};

            cluster.push_back(init_XY);
            for (int i = 1; i< size_laser; i++)
            {
                double old_range, new_range;
                old_range = laserData.ranges.at(i-1);
                new_range = laserData.ranges.at(i);
                
                // if the new range is smaller than the laser min, check cluster size, if any, then break out of current angle.
                if ( (new_range <= min_laserScan_range) && ((i+1)!=size_laser) )
                {
                    if (cluster.size()>=4){
                        main_cluster.push_back(cluster);
                        cluster.clear();
                    }
                }
                else
                {
                    turtlelib::Vector2D old_xy, new_xy;
                    old_xy.x = old_range * cos(turtlelib::deg2rad(i-1));
                    old_xy.y = old_range * sin(turtlelib::deg2rad(i-1));
                    new_xy.x = new_range * cos(turtlelib::deg2rad(i));
                    new_xy.y = new_range * sin(turtlelib::deg2rad(i));
                    //only for the very first cluster and at angle 0
                    if (cluster.size() == 0)
                    {
                        cluster.push_back(old_xy);
                    }

                    if (fabs(new_range-old_range) < cluster_thresh) 
                    {
                        cluster.push_back(new_xy);
                    }
                    //found a point outside of the current cluster
                    else
                    {
                        if (cluster.size()>=4){
                            main_cluster.push_back(cluster);
                        }
                        if (main_cluster.size()==0){
                            main_cluster.push_back(cluster);
                        }
                        //try populatinig with something.
                        cluster.clear();
                        if ((i+1) == size_laser){
                            cluster.push_back(new_xy);
                        }
                    }
                }  
            }

            double init_range = laserData.ranges.at(0);
            // turtlelib::Vector2D init_xy;
            // double delta_x,delta_y;
            // delta_x = (init_range*cos(0))-(laserData.ranges.at(size_laser-1)*cos(turtlelib::deg2rad(size_laser-1)));
            // delta_y = (init_range*sin(0))-(laserData.ranges.at(size_laser-1)*sin(turtlelib::deg2rad(size_laser-1)));
            
            double range_before_start;
            range_before_start = laserData.ranges.at((size_laser-1));
            std::cout << "___BEFORE__MAIN CLUSTER SIZE____"<< main_cluster.size() << std::endl;
            // if ( turtlelib::almost_equal(delta_x,0.0,0.1) && turtlelib::almost_equal(delta_y,0.0,0.1))
            // {
            if ((fabs(range_before_start-init_range) < cluster_thresh) && range_before_start>0.1)
            {
                // cluster.print();
                if (cluster.size()>0){
                    main_cluster[0].insert(main_cluster[0].begin(),cluster.begin(),cluster.end());
                    cluster.clear();
                }
                if (cluster.size()>=4)
                {
                    main_cluster.push_back(cluster);
                }
                if (main_cluster[0].size() < 4 && main_cluster.size() > 0)
                {
                    main_cluster.erase(main_cluster.begin());
                } 
            }
            else
            {
                (cluster.size()>0);
                cluster.clear();
            }
            
                 

            // }
            
                      
            std::cout << "_____MAIN CLUSTER SIZE____"<< main_cluster.size() << std::endl;

            std::vector<std::vector<turtlelib::Vector2D>> true_clusters;
            makeCircles = nuslam::nuCircles(main_cluster);
            true_clusters = makeCircles.ClassifyCircles();
            makeCircles = nuslam::nuCircles(true_clusters);
            showCluster();
            makeCircles.CircleFitting();
            R_array = makeCircles.getR_array();
            xy_COORDS = makeCircles.getCirclePosition();
            showObstacles();
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
    ros::Publisher obstacle_marker_pub;
    ros::Timer timer;
    
    //create variable for cluster distance
    double cluster_thresh;

    double min_laserScan_range;
    visualization_msgs::MarkerArray cluster_marker;
    visualization_msgs::MarkerArray obstacle_marker;
    std::vector<std::vector<turtlelib::Vector2D>> main_cluster;
    std::vector<std::vector<turtlelib::Vector2D>> true_clusters;
    std::vector<double> R_array;
    std::vector<turtlelib::Vector2D> xy_COORDS;

    nuslam::nuCircles makeCircles;

    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "LANDMARKS");
    Landmarks node;
    ros::spin();
    return 0;
}