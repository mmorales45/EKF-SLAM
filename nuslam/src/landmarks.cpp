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
            obstacle_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/points/Obstacles", 1, true);

            cluster_thresh = 0.2;
            min_laserScan_range = 0.05;
        }

        
        void showObstacles()
        {
            int counter = 0;
            int size_main = main_cluster.size();
            // std::cout << "________GOT TO HERE 0____________" << std::endl;
            // visualization_msgs::MarkerArray cluster_marker;
            // std::cout << "________GOT TO HERE 1____________" << std::endl;
            obstacle_marker.markers.resize(size_main);
            for (int i = 0;i<size_main;i++)
            {
                obstacle_marker.markers[i].header.stamp = ros::Time();
                obstacle_marker.markers[i].header.frame_id = "red-base_footprint";
                uint32_t shape;
                shape = visualization_msgs::Marker::CYLINDER;
                obstacle_marker.markers[i].type = shape;
                obstacle_marker.markers[i].ns = "Obstacles";
                obstacle_marker.markers[i].action = visualization_msgs::Marker::ADD;
                obstacle_marker.markers[i].id = i;

                obstacle_marker.markers[i].pose.position.x = xy_COORDS.at(i).x;
                obstacle_marker.markers[i].pose.position.y = xy_COORDS.at(i).y;
                obstacle_marker.markers[i].pose.position.z = 0.2;
                obstacle_marker.markers[i].pose.orientation.x = 0.0;
                obstacle_marker.markers[i].pose.orientation.y = 0.0;
                obstacle_marker.markers[i].pose.orientation.z = 0.0;
                obstacle_marker.markers[i].pose.orientation.w = 1.0;

                obstacle_marker.markers[i].scale.x = R_array.at(i)/2;
                obstacle_marker.markers[i].scale.y = R_array.at(i)/2;
                obstacle_marker.markers[i].scale.z = (0.05);
                
                obstacle_marker.markers[i].color.r = 1.0;
                obstacle_marker.markers[i].color.g = 1.0;
                obstacle_marker.markers[i].color.b = 0.0;
                obstacle_marker.markers[i].color.a = 1.0;
            }
            // std::cout << "________GOT TO HERE 3___________" << std::endl;
            obstacle_marker_pub.publish(obstacle_marker); 
            // std::cout << "________GOT TO HERE 4___________" << std::endl;
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
                // std::cout << "________GOT TO HERE 2___________" << std::endl;

            }
            // std::cout << "________GOT TO HERE 3___________" << std::endl;
            cluster_marker_pub.publish(cluster_marker); 
            // std::cout << "________GOT TO HERE 4___________" << std::endl;
        }

        void ClassifyCircles()
        {
            std::cout << "________GOT TO HERE 0____________" << std::endl;
            double min_angle = turtlelib::PI/2;
            double max_angle = (3*turtlelib::PI)/2;
            std::vector<std::vector<turtlelib::Vector2D>> new_Clusters;
            int size_main = main_cluster.size();
            std::cout << "________GOT TO HERE 1____________" << std::endl;
            for (int i = 0;i<size_main;i++)
            {
                std::vector<turtlelib::Vector2D> currentCluster = main_cluster[i];
                int size_cluster = currentCluster.size();
                turtlelib::Vector2D Point_1,Point_2;
                Point_1 = currentCluster[0];
                Point_2 = currentCluster[size_cluster-1];

                double angle_sums = 0.0;
                std::vector<double> angle_array;
                std::cout << "________GOT TO HERE 3___________" << std::endl;
                for (int j = 1;j< (size_cluster-1);j++)
                {
                    turtlelib::Vector2D new_Point1, new_Point2;
                    new_Point1 = {(Point_1.x - currentCluster[j].x),
                                    (Point_1.y - currentCluster[j].y)};
                    new_Point2 = {(Point_2.x - currentCluster[j].x),
                                    (Point_2.y - currentCluster[j].y)};
                    double arc_angle = turtlelib::angle(new_Point1,new_Point2);
                    angle_sums = angle_sums + arc_angle;
                    angle_array.push_back(arc_angle);
                }
                std::cout << "________GOT TO HERE 4____________" << std::endl;
                double angle_mean = 0.0;
                angle_mean = angle_sums / (size_cluster-2);        

                double standard_deviation = 0.0;
                for (int k = 0;k<(size_cluster-2);k++)
                {
                    double sd_val = pow( (angle_array.at(k)-angle_mean) ,2 );
                    standard_deviation = standard_deviation + sd_val;
                }
                double sd_compare = 0.0;
                sd_compare = standard_deviation/((size_cluster-2));

                if ((angle_mean <= max_angle) && (angle_mean >= min_angle) && (sd_compare<0.15)) 
                {
                    new_Clusters.push_back(currentCluster);
                }   
                std::cout << "________GOT TO HERE 5____________" << std::endl;
            }
            true_clusters =  new_Clusters;
            std::cout << "________GOT TO HERE 6____________" << std::endl;
        }

        
        void CircleFitting()
        {
            // std::cout << "________GOT TO HERE -1____________" << std::endl;
            int size_main = main_cluster.size();
            R_array.clear();
            xy_COORDS.clear();
            // std::vector<double> R_array;
            // std::vector<turtlelib::Vector2D> xy_COORDS;
            R_array.resize(size_main);
            xy_COORDS.resize(size_main);
            // std::cout << "_num of clusters"<< size_main <<"____________" << std::endl;
            for (int i = 0;i<size_main;i++)
            {
                //Create a cluster to loop at from the main set of clusters.
                std::vector<turtlelib::Vector2D> currentCluster = main_cluster[i];
                int size_cluster = currentCluster.size();
                // std::cout << "_num of points in cluster"<< size_cluster <<"____________" << std::endl;

                double x_sums = 0.0;
                double y_sums = 0.0;
                //loop through every cluster to calculate the sums of both x and y values.
                for (int j = 0;j< size_cluster;j++)
                {
                    turtlelib::Vector2D currentXY = currentCluster[j];
                    x_sums = x_sums + currentXY.x;
                    y_sums = y_sums + currentXY.y;
                }
                //calculate means of both x and y.
                double x_mean,y_mean;
                x_mean = x_sums/size_cluster;
                y_mean = y_sums/size_cluster;

                //Create empty vectors to append the centroid values.
                std::vector<double> x_cent;
                std::vector<double> y_cent;
                std::vector<double> z_cent;
                x_cent.resize(size_cluster);
                y_cent.resize(size_cluster);
                z_cent.resize(size_cluster);

                //Loop through every value to calculate zi and centroid values.
                double z_sums = 0;
                for (int j = 0;j< size_cluster;j++)
                {
                    turtlelib::Vector2D currentXY = currentCluster[j];
                    double x_diff, y_diff, z_i;
                    x_diff = currentXY.x - x_mean;
                    y_diff = currentXY.y - y_mean;
                    z_i = pow(x_diff,2) + pow(y_diff,2);

                    x_cent.at(j) = x_diff;
                    y_cent.at(j) = y_diff;
                    z_cent.at(j) = z_i;
                    z_sums = z_sums + z_i;
                }

                //calculate z mean.
                double z_mean = 0;
                z_mean = z_sums/size_cluster;
                // std::cout << "________GOT TO HERE 0____________" << std::endl;
                //create Z matrix and loop through every row to fill up each cell.
                arma::mat Z(size_cluster,4,arma::fill::ones);
                for (int k = 0;k< size_cluster;k++)
                {
                    Z(k,0) = z_cent.at(k);
                    Z(k,1) = x_cent.at(k);
                    Z(k,2) = y_cent.at(k);
                    Z(k,3) = 1;
                }

                //create M matrix.
                arma::mat M(4,4,arma::fill::zeros);
                // std::cout << "________GOT TO HERE T1____________" << std::endl;
                M = (1/size_cluster) * Z.t() * Z;
                // std::cout << "________GOT TO HERE T2____________" << std::endl;
                //create H matrix
                arma::mat H(4,4,arma::fill::zeros);
                H = {   {8*z_mean, 0, 0, 2},
                        {0, 1, 0, 0},
                        {0, 0, 1, 0},
                        {2, 0, 0, 0} };
                // create Hinv matrix
                arma::mat H_inv(4,4,arma::fill::zeros);
                H_inv = {   {0, 0, 0, 0.5},
                            {0, 1, 0, 0},
                            {0, 0, 1, 0},
                            {0.5, 0, 0, -2*z_mean} };
                // std::cout << "________GOT TO HERE T3____________" << std::endl;
                //calculate svd using arma
                arma::mat U;
                arma::vec sigma; //cant be a mat, needs to be vec
                arma::mat V;
                arma::svd(U,sigma,V,Z);
                // std::cout << "________GOT TO HERE T4____________" << std::endl;
                // sigma.print();
                // The smallest sigma is the 4th column
                double smallest_sigma = sigma(3);
                // std::cout << "________GOT TO HERE T5____________" << std::endl;
                //if sigma is less than 0.0001 set A to
                // std::cout << "________GOT TO HERE 1____________" << std::endl;
                arma::vec A;
                if (smallest_sigma < 1.0e-4)
                {
                    A = V.col(3); //make A equal to 4th colum
                }
                else
                {
                    arma::mat Y;
                    Y = V * diagmat(sigma) * V.t();
                    arma::mat Q;
                    Q = Y * H_inv * Y;

                    arma::vec eigval;
                    arma::mat eigvec;

                    eig_sym(eigval,eigvec,Q);

                    // std::cout << "_____________"<< i << "_______" << std::endl;
                    // eigval.print();
                    // std::cout << "____________________" << std::endl;
                    // eigvec.print();
                    // double A_star = 0;
                    arma::vec A_star_vec;
                    double eig_compare = 1000.0;
                    int iter = 0;
                    for (int k = 0;k<eigval.size();k++)
                    {
                        if ((eigval[k]>0.0 && eigval[k]<eig_compare))
                        {
                            iter = k;
                            eig_compare = eigval[k];
                        }
                    }
                    A_star_vec = eigvec.col(iter);
                    // std::cout << "SMALLEST" << std::endl;
                    // std::cout << eig_compare << std::endl;
                    
                    A = arma::solve(Y,A_star_vec);

                    // std::cout << "____________________" << std::endl;
                    // A.print();
                }
                // std::cout << "________GOT TO HERE 2____________" << std::endl;
                // A.print();
                double a,b,R;
                a = -A(1)/(2*A(0));
                b = -A(2)/(2*A(0));
                // std::cout << "________GOT TO HERE 3____________" << std::endl;
                R = sqrt( (pow(A(1),2) + pow(A(2),2) - (4*A(0)*A(3)))/(4*pow(A(0),2)) );
                
                turtlelib::Vector2D circleCenter = {a+x_mean,b+y_mean};
                R_array.at(i) = R;
                xy_COORDS.at(i) = circleCenter;
            }
        }

        /// \brief Callback for the fake sensor data topic.
        ///
        void ls_callback(const sensor_msgs::LaserScan & laserData)
        {
            // std::cout << "________GOT TO HERE 0____________" << std::endl;
            //clear the main cluster every run
            main_cluster.clear();
            // std::cout << "________GOT TO HERE 1____________" << std::endl;
            // std::vector<std::vector<turtlelib::Vector2D>> main_cluster;
            //get the size of the laser scan array
            int size_laser = laserData.ranges.size();
            std::vector<turtlelib::Vector2D> cluster;

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
                // std::cout << "________GOT TO HERE 3____________" << std::endl;
                if ( (fabs(new_range-old_range) < cluster_thresh)&& ((i+1)!=size_laser) && (new_range>0.1)) 
                {
                    ;
                    
                }
                else
                {
                    if (cluster.size()>=4){
                        main_cluster.push_back(cluster);
                    }
                    cluster.clear();
                }

                turtlelib::Vector2D old_xy, new_xy;
                old_xy.x = old_range * cos(turtlelib::deg2rad(i-1));
                old_xy.y = old_range * sin(turtlelib::deg2rad(i-1));
                new_xy.x = new_range * cos(turtlelib::deg2rad(i));
                new_xy.y = new_range * sin(turtlelib::deg2rad(i));;

                cluster.push_back(new_xy);

                int size_main = cluster.size();
                std::cout << "_____CLUSTER SIZE_______"<< size_main << "_____AT____"<<i<<std::endl;
            }
            

            int size_main = main_cluster.size();
            std::cout << "_____BEFORE_______"<< size_main << "____________"<<std::endl;
            //check for the loop closure
            if ( (laserData.ranges.at(0) > 0) && (laserData.ranges.at(1) > 0) )
            {
                if ((fabs(laserData.ranges.at(0) - laserData.ranges.at(1))) < cluster_thresh)
                {
                    std::cout << "_____ERASING________"<<std::endl;
                    main_cluster[0].insert(main_cluster[0].end(),main_cluster.at(size_main-1).begin(),main_cluster.at(size_main-1).end());
                    main_cluster.pop_back();
                }
            }
            size_main = main_cluster.size();
            std::cout << "_____AFTER_______"<< size_main << "____________"<<std::endl;
            ClassifyCircles();
            showCluster();
            CircleFitting();
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

    
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "LANDMARKS");
    Landmarks node;
    ros::spin();
    return 0;
}