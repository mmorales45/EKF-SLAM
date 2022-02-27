/// \file
/// \brief This file creates custom services that affect the turtlebot as well as create obstacles in RVIZ.
///
/// PARAMETERS:
///     rate (double): Frequency at which the main loop runs
///     cylinders_x_coord (std::vector<double>): The x coordaintes of the three red cylinders
///     cylinders_y_coord (std::vector<double>): The y coordaintes of the three red cylinders
///     robot (std::vector<double>): vector/array of the robot's iniital coordinates in x,y,and theta
///     radius (double): the radius of the red cylinders
/// SUBSCRIBERS:
///     wheel_cmd_sub (nuturtlebot_msgs/WheelCommands): Get the wheels commands for converting to encorder data
/// PUBLISHES:
///     timestep_pub (std_msgs/UInt64): Tracks the simulation's current timestep
///     joint_state_pub (sensor_msgs/JointState): Publishes joint positions to red/joint_states
///     marker_pub (visualization_msgs/MarkerArray): Creates obstacles as red cylinders in RVIZ
///     walls_pub (visualization_msgs/MarkerArray): Creates walls
///     encoder_pub (nuturtlebot_msgs/SensorData): Publish the encorder data
///     path_pub (nav_msgs/Path): Create the path that the red(simulated) robot follows
///     fake_sensor_pub (sensor_msgs/LaserScan): Create the fake obstacles relative to the robot
/// SERVICES:
///     reset (nusim/reset): Sets the timestep to 0 and teleports turtlebot to the initial position
///     teleport (nusim/Teleport): Teleports the turtblebot to a user defined position

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
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cstdlib>

#include <turtlelib/rigid2d.hpp>
#include <turtlelib/diff_drive.hpp>

#include <nuturtlebot_msgs/WheelCommands.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include<random>

/// \brief Create a struct that holds the two sets of intersection points for obstacles.
/// \returns A struct composed of four double points.
struct intersection_points {
    double x1;
    double x2;
    double y1;
    double y2;
};

/// \brief Create a struct that holds the two sets of intersection points for the wall.
/// \returns A struct composed of four double points.
struct wall_points {
    double x1;
    double x2;
    double y1;
    double y2;
};

/// \brief Creates a simulator and visualizer for the turtlebot3
class Sim
{
    public:
        Sim() {
            //get parameters from basic_world.yaml
            timestep.data = 0;
            nh.getParam("/nusim/rate",rate);
            nh.getParam("/nusim/cylinders_x_coord",cylinders_x_coord);
            nh.getParam("/nusim/cylinders_y_coord",cylinders_y_coord);
            nh.getParam("/nusim/robot",robot_coords);
            nh.getParam("/nusim/radius",radius);
            nh.getParam("/nusim/x_length",x_length);
            nh.getParam("/nusim/y_length",y_length);
            nh.getParam("/nusim/motor_cmd_to_radsec",motor_cmd_to_radsec);
            nh.getParam("/nusim/encoder_ticks_to_rad",encoder_ticks_to_rad);
            nh.getParam("/nusim/motor_cmd_max",motor_cmd_max);
            nh.getParam("/nusim/sim_flag",sim_flag);
            nh.getParam("/nusim/slip_min",slip_min);
            nh.getParam("/nusim/slip_max",slip_max);
            nh.getParam("/nusim/max_range",max_range);
            nh.getParam("/nusim/basic_sensor_variance",basic_sensor_variance);
            nh.getParam("/nusim/samples",samples);
            nh.getParam("/nusim/resolution",resolution);
            nh.getParam("/nusim/min_angle",min_angle);
            nh.getParam("/nusim/max_angle",max_angle);
            nh.getParam("/nusim/scan_time",scan_time);
            nh.getParam("/nusim/min",min_scan_range);
            nh.getParam("/nusim/max",max_scan_range);
            nh.getParam("/nusim/robot_noise_mean",robot_noise_mean);
            nh.getParam("/nusim/robot_noise_stddev",robot_noise_stddev);
            nh.getParam("/nusim/collision_radius",collision_radius);
            nh.getParam("/nusim/laser_variance",laser_variance);
            
            motor_cmd_max_lower = motor_cmd_max[0];
            motor_cmd_max_upper = motor_cmd_max[1];
            

            //Initialize the timer, services, and publishers
            timestep_pub = nh.advertise<std_msgs::UInt64>("/nusim/timestep", 100);
            marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nusim/obstacles/markerArray", 1, true);
            fake_marker_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nusim/obstacles/Fake_markerArray", 1, true);
            walls_pub  = nh.advertise<visualization_msgs::MarkerArray>("/nusim/walls/walls", 1, true);
            encoder_pub  = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", 10, true);
            wheel_cmd_sub = nh.subscribe("red/wheel_cmd", 100, &Sim::wheel_cmd_callback, this);
            path_pub  = nh.advertise<nav_msgs::Path>("nusim/path", 10, true);
            fake_sensor_pub  = nh.advertise<sensor_msgs::LaserScan>("nusim/laser_scan_data", 10, true);
        
            
            
            reset_service = nh.advertiseService("nusim/reset", &Sim::reset, this);
            teleport_service = nh.advertiseService("nusim/teleport", &Sim::teleport, this);
            make_arena();
            timer = nh.createTimer(ros::Duration(1/rate), &Sim::main_loop, this);
            timer_fake = nh.createTimer(ros::Duration(1.0/5.0), &Sim::fake_loop, this);

            //set initial coordinates for the robot as well as the header and child ids
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "red-base_footprint";
            current_config.x = robot_coords[0];
            current_config.y = robot_coords[1];
            current_config.theta = robot_coords[2];
            q.setRPY(0, 0, current_config.theta);
            //add values to x and y coordiante vectors
            obstacle_array_size = (cylinders_x_coord.size());
            for (int j = 0;j<obstacle_array_size;j++){
                cylinder_marker_x.push_back(cylinders_x_coord[j]);
                cylinder_marker_y.push_back(cylinders_y_coord[j]);
            }

            DiffDrive = turtlelib::DiffDrive(current_config,wheel_angles,wheels_velocity);
            marker.markers.resize(obstacle_array_size);
            for (int i = 0;i<obstacle_array_size;i++)
            {
                marker.markers[i].header.stamp = ros::Time();
                marker.markers[i].header.frame_id = "world";
                shape = visualization_msgs::Marker::CYLINDER;
                marker.markers[i].type = shape;
                marker.markers[i].ns = "obstacles";
                marker.markers[i].action = visualization_msgs::Marker::ADD;
                marker.markers[i].id = i;

                marker.markers[i].pose.position.x = cylinder_marker_x[i];
                marker.markers[i].pose.position.y = cylinder_marker_y[i];
                marker.markers[i].pose.position.z = 0.125;
                marker.markers[i].pose.orientation.x = 0.0;
                marker.markers[i].pose.orientation.y = 0.0;
                marker.markers[i].pose.orientation.z = 0.0;
                marker.markers[i].pose.orientation.w = 1.0;

                marker.markers[i].scale.x = (2*radius);
                marker.markers[i].scale.y = (2*radius);
                marker.markers[i].scale.z = 0.25;
                
                marker.markers[i].color.r = 1.0;
                marker.markers[i].color.g = 0.0;
                marker.markers[i].color.b = 0.0;
                marker.markers[i].color.a = 1.0;
            }
            marker_pub.publish(marker);
            // std::normal_distribution<> left_vel_noise(0, .01);
            // std::normal_distribution<> right_vel_noise(0, .01);
            // std::normal_distribution<> slip(0.5, slip_max);
            // // std::normal_distribution<> fake_obstacle_noise(0, basic_sensor_variance);
            // left_noise = left_vel_noise(get_random());
            // right_noise = right_vel_noise(get_random());
            // slip_noise = slip(get_random());
            // obstacle_noise = fake_obstacle_noise(get_random());

            collision_flag = false;
        }

        std::mt19937 & get_random()
        {
            // static variables inside a function are created once and persist for the remainder of the program
            static std::random_device rd{}; 
            static std::mt19937 mt{rd()};
            // we return a reference to the pseudo-random number genrator object. This is always the
            // same object every time get_random is called
            return mt;
        }

        /// \brief sets the timestep to 0 and teleports robot to intial position
        ///
        /// \param data - empty
        /// \returns response - true 
        bool reset(std_srvs::Empty::Request& , std_srvs::Empty::Response& )
        {
            timestep.data = 0;
            timestep_pub.publish(timestep);
            current_config.x = robot_coords[0];
            current_config.y = robot_coords[1];
            theta = robot_coords[2];
            current_config.theta = theta;
        return true;
        }

        /// \brief teleports the robot to a position set by the user
        ///
        /// \param data - Teleport data type composed of user x,y,theta
        /// \returns response - true 
        bool teleport(nusim::Teleport::Request& data, nusim::Teleport::Response& )
        {
            current_config.x = data.x;
            current_config.y = data.y;
            current_config.theta = data.theta;
            theta = data.theta;
            return true;
        }

        /// \brief Creates arena for the turtlebot3 by using 4 walls
        ///
        void make_arena()
        {
            x_walls.markers.resize(4);
            for (int i = 0;i<4;i++)
            {
                if (i ==0)
                {
                    x_walls.markers[i].pose.position.x = 0;
                    x_walls.markers[i].pose.position.y = -y_length/2 ;
                    x_walls.markers[i].scale.x = x_length +0.1;
                    x_walls.markers[i].scale.y = 0.1;
    
                }
                if (i ==1)
                {
                    x_walls.markers[i].pose.position.x = x_length/2;
                    x_walls.markers[i].pose.position.y = 0;
                    x_walls.markers[i].scale.x = 0.1;
                    x_walls.markers[i].scale.y = y_length+0.1;
    
                }
                if (i ==2)
                {
                    x_walls.markers[i].pose.position.x = 0;
                    x_walls.markers[i].pose.position.y = y_length/2;
                    x_walls.markers[i].scale.x = x_length+0.1;
                    x_walls.markers[i].scale.y = 0.1;
    
                }
                if (i ==3)
                {
                    x_walls.markers[i].pose.position.x = -x_length/2;
                    x_walls.markers[i].pose.position.y = 0;
                    x_walls.markers[i].scale.x = 0.1;
                    x_walls.markers[i].scale.y = y_length+0.1;
    
                }
                x_walls.markers[i].header.stamp = ros::Time();
                x_walls.markers[i].header.frame_id = "world";
                shape = visualization_msgs::Marker::CUBE;
                x_walls.markers[i].type = shape;
                x_walls.markers[i].ns = "walls";
                x_walls.markers[i].action = visualization_msgs::Marker::ADD;
                x_walls.markers[i].id = i;

                x_walls.markers[i].pose.position.z = 0.125;

                x_walls.markers[i].pose.orientation.x = 0.0;
                x_walls.markers[i].pose.orientation.y = 0.0;
                x_walls.markers[i].pose.orientation.z = 0.0;
                x_walls.markers[i].pose.orientation.w = 1.0;

    
                x_walls.markers[i].scale.z = 0.25;
                
                x_walls.markers[i].color.r = 1.0;
                x_walls.markers[i].color.g = 0.0;
                x_walls.markers[i].color.b = 0.0;
                x_walls.markers[i].color.a = 1.0;
            }
            walls_pub.publish(x_walls);

         
        }
        /// \brief Creates obstacles with noise relative to the robot
        void make_fake_obstacles()
        {
            turtlelib::Vector2D red_trans;
            double red_angle;
            red_trans.x = current_config.x;
            red_trans.y = current_config.y;
            red_angle = current_config.theta;
            turtlelib::Transform2D T_WR;
            T_WR = turtlelib::Transform2D(red_trans,red_angle);
            turtlelib::Transform2D T_RW;
            T_RW = T_WR.inv();
            
            fake_marker.markers.resize(obstacle_array_size);
            for (int i = 0;i<obstacle_array_size;i++)
            {
                std::normal_distribution<> fake_obstacle_noise(0, basic_sensor_variance);
                obstacle_noise = fake_obstacle_noise(get_random());
                turtlelib::Vector2D obs_trans;
                obs_trans.x = cylinder_marker_x[i]+obstacle_noise;
                obs_trans.y = cylinder_marker_y[i]+obstacle_noise;
                turtlelib::Transform2D T_WO;
                T_WO = turtlelib::Transform2D(obs_trans);
                turtlelib::Transform2D T_RO;
                T_RO = T_RW*T_WO;
                turtlelib::Vector2D new_obs = T_RO.translation();

                fake_marker.markers[i].header.stamp = ros::Time();
                fake_marker.markers[i].header.frame_id = "red-base_footprint";
                shape = visualization_msgs::Marker::CYLINDER;
                fake_marker.markers[i].type = shape;
                fake_marker.markers[i].ns = "fake_obstacles";

                fake_marker.markers[i].id = i;

                fake_marker.markers[i].pose.position.x = new_obs.x;
                fake_marker.markers[i].pose.position.y = new_obs.y;
                fake_marker.markers[i].pose.position.z = 0.125;
                fake_marker.markers[i].pose.orientation.x = 0.0;
                fake_marker.markers[i].pose.orientation.y = 0.0;
                fake_marker.markers[i].pose.orientation.z = 0.0;
                fake_marker.markers[i].pose.orientation.w = 1.0;

                fake_marker.markers[i].scale.x = (2*radius);
                fake_marker.markers[i].scale.y = (2*radius);
                fake_marker.markers[i].scale.z = 0.25;
                
                fake_marker.markers[i].color.r = 0.0;
                fake_marker.markers[i].color.g = 0.0;
                fake_marker.markers[i].color.b = 1.0;
                fake_marker.markers[i].color.a = 1.0;
                fake_marker.markers[i].frame_locked = true;
                // distance = sqrt(pow(cylinder_marker_x[i]-current_config.x,2)+(cylinder_marker_y[i]-current_config.y,2));                
                distance = get_distance(current_config.x,current_config.y,cylinder_marker_x[i],cylinder_marker_y[i]);
                if (distance < max_range){
                    fake_marker.markers[i].action = visualization_msgs::Marker::ADD;
                }
                else{
                    fake_marker.markers[i].action = visualization_msgs::Marker::DELETE;
                }
            }
            fake_marker_pub.publish(fake_marker);
            
        }

        /// \brief Calculate the distance between 
        ///
        /// \param x1 - x-coordinate of the first point.
        /// \param y1 - y-coordinate of the first point.
        /// \param x2 - x-coordinate of the second point.
        /// \param y2 - y-coordinate of the second point.
        ///
        /// \returns calc_distance - Calculate the distance. 
        double get_distance(double x1, double y1, double x2, double y2)
        {
            double calc_distance;
            calc_distance = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
            return calc_distance;
        }

        /// \brief Check to see if the robot collided with an obstacle.
        void check_for_collision()
        {
            double collision_distance;
            for (int i = 0;i<obstacle_array_size;i++)
            {
                collision_distance = get_distance(current_config.x,current_config.y,cylinder_marker_x[i],cylinder_marker_y[i]);
                if ((collision_distance<(collision_radius+radius)) && (collision_flag==false))
                {   
                    collision_flag=true;

                    // ROS_WARN("COLLISION!");
                }

                if ((collision_distance>collision_radius+radius))
                {   

                    collision_flag=false;

                }

                if (collision_flag==true)
                {
                    current_config.x = collision_x;
                    current_config.y = collision_y;
                    current_config.theta = collision_theta;
                    // collision_flag=false;
                }
            }
        }

        /// \brief Calculate distance between intersections.
        ///
        /// \param dx - Change in x.
        /// \param dy - Change iny.
        ///
        /// \returns dr - Distance between intersections.
        double get_dr(double dx, double dy)
        {
            double dr;
            dr = sqrt(pow(dx,2)+pow(dy,2));
            return dr;
        }

        /// \brief Calculate determinant.
        ///
        /// \param x1 - x-coordinate of the first point.
        /// \param y1 - y-coordinate of the first point.
        /// \param x2 - x-coordinate of the second point.
        /// \param y2 - y-coordinate of the second point.
        ///
        /// \returns D - The determinant.
        double get_D(double x1,double y1,double x2, double y2)
        {
            double D;
            D = x1*y2-x2*y1;
            return D;
        }

        /// \brief Calculate if sgn if 1 or -1.
        ///
        /// \param var - variable to check if sgn is pos or neg.
        ///
        /// \returns sgn - (1) or -1.
        double get_sgn(double var)
        {
            double sgn;
            if (var<0)
            {
                sgn = -1;
            }
            else{
                sgn = 1;
            }
            return sgn;
        }

        /// \brief Calculate coordinates of intersection.
        ///
        /// \param D - The determinant.
        /// \param dx - Change in x.
        /// \param dy - Change in y.
        /// \param r - Radius of obstacle.
        /// \param dr - Distance between intersection points.
        ///
        /// \returns points_of_interest - Intersection points.
        intersection_points get_points_intersection(double D, double dx, double dy,double r,double dr)
        {
            intersection_points points_of_interest;
            // double x1,y1,x2,y2;
            points_of_interest.x1 = (D*dy+get_sgn(dy)*dx*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));
            points_of_interest.y1 = (-D*dx+fabs(dy)*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));
            points_of_interest.x2 = (D*dy-get_sgn(dy)*dx*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));
            points_of_interest.y2 = (-D*(dx)-fabs(dy)*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));

            return points_of_interest;
        }

        /// \brief Calculate discriminant.
        ///
        /// \param D - The determinant.
        /// \param r - Radius of obstacle.
        /// \param dr - Distance between intersection points.
        ///
        /// \returns discriminant - Determining incidence of the line and circle.
        double get_discriminant(double D, double r, double dr)
        {
            double discriminant;
            discriminant = pow(r,2)*pow(dr,2)-pow(D,2);
            return discriminant;
        }

        /// \brief Create line.
        ///
        /// \param x1 - x intersection of point 1.
        /// \param y1 - y intersection of point 1.
        /// \param x2 - x intersection of point 2.
        /// \param y2 - y intersection of point 2.
        ///
        /// \returns slope - Slope between points.
        double create_line(double x1,double y1,double x2,double y2)
        {
            double slope = (y2-y1)/(x2-x1);
            return slope;
        }
        
        /// \brief Creates a vector of wall points.
        ///
        std::vector<wall_points> create_walls()
        {
            wall_points wall_left,wall_bot,wall_right,wall_top;
            std::vector<wall_points> new_walls;
            new_walls.resize(4);
            wall_left.x1 = -x_length/2 +0.1;
            wall_left.y1 = -y_length/2;
            wall_left.x2 = -x_length/2 +0.1;
            wall_left.y2 = y_length/2;
            new_walls[2] = wall_left;

            wall_bot.x1 = -x_length/2;
            wall_bot.y1 = -y_length/2 +0.1;
            wall_bot.x2 = x_length/2;
            wall_bot.y2 = -y_length/2 +0.1;
            new_walls[3] = wall_bot;
            
            wall_right.x1 = x_length/2 -0.1;
            wall_right.y1 = -y_length/2;
            wall_right.x2 = x_length/2 -0.1;
            wall_right.y2 = y_length/2;
            new_walls[0] = wall_right;

            wall_top.x1 = -x_length/2;
            wall_top.y1 = y_length/2 -0.1;
            wall_top.x2 = x_length/2;
            wall_top.y2 = y_length/2 -0.1;
            new_walls[1] = wall_top;

            return new_walls;

        }

        /// \brief Creates fake laser data points for obstacles and walls.
        void make_fake_laser()
        {
            std::normal_distribution<> laser_noise(0, laser_variance);
            turtlelib::Vector2D min_range_vector;
            turtlelib::Vector2D max_range_vector;
            double scan_angle;
            double D, discriminant;
            double dx,dy;
            double dx_infinite,dy_infinite;
            double dr;
            intersection_points intersections;

            turtlelib::Vector2D red_trans;
            turtlelib::Vector2D red_trans_in_obstacle;
            double red_angle;
            red_trans.x = current_config.x;
            red_trans.y = current_config.y;
            red_angle = current_config.theta;
            turtlelib::Transform2D T_WR;
            T_WR = turtlelib::Transform2D(red_trans,red_angle);
            turtlelib::Transform2D T_RW;
            T_RW = T_WR.inv();
            std::vector<turtlelib::Vector2D> obstacle_robot_obs;
            std::vector<turtlelib::Vector2D> robot_obs_robot;
            std::vector<double> distance_list;
            obstacle_robot_obs.resize(obstacle_array_size);
            robot_obs_robot.resize(obstacle_array_size);
            double points_compare_x;
            double points_compare_y;    

            // initialize components of the laser message
            fake_laser.header.frame_id = "red-base_scan";
            fake_laser.header.stamp = ros::Time::now();
            fake_laser.angle_min = min_angle;
            fake_laser.angle_max = max_angle;
            fake_laser.angle_increment = max_angle/360;
            fake_laser.time_increment = 0.5/1000000000.0;
            fake_laser.scan_time = scan_time/1000000000.0;
            fake_laser.range_min = min_scan_range;
            fake_laser.range_max = max_scan_range;
            fake_laser.ranges.resize(samples);
            for (int i = 0;i<samples;i++)
            {
                std::vector<double> distance_list;
                double new_distance1;
                double new_distance2;
                scan_angle = turtlelib::deg2rad(i);
                min_range_vector.x = min_scan_range*cos(scan_angle);
                min_range_vector.y = min_scan_range*sin(scan_angle);
                max_range_vector.x = max_scan_range*cos(scan_angle);
                max_range_vector.y = max_scan_range*sin(scan_angle);
                turtlelib::Vector2D first_point = (min_range_vector);
                turtlelib::Vector2D second_point = (max_range_vector);
                dx_infinite = (second_point.x - first_point.x);
                dy_infinite = (second_point.y - first_point.y);
                dr = get_dr(dx_infinite,dy_infinite);



                for (int j = 0;j<obstacle_array_size;j++)
                {
                    turtlelib::Vector2D test_vec;
                    turtlelib::Vector2D test_vec2;
                    turtlelib::Vector2D obstacle_translational;
                    obstacle_translational.x = cylinder_marker_x[j];
                    obstacle_translational.y = cylinder_marker_y[j];
                    turtlelib::Transform2D T_WO;
                    T_WO = turtlelib::Transform2D(obstacle_translational);
                    turtlelib::Transform2D T_OW;
                    T_OW = T_WO.inv();
                    turtlelib::Transform2D T_OR;
                    T_OR = T_OW*T_WR;
                    turtlelib::Transform2D T_RO;
                    T_RO = T_OR.inv();
                    turtlelib::Vector2D new_obs = T_OR.translation();
                    obstacle_robot_obs[j] = new_obs;


                    new_distance1 = 0;
                    new_distance2 = 0;
                    test_vec = T_OR(first_point);
                    test_vec2 = T_OR(second_point);

                    D = get_D(test_vec.x,test_vec.y,test_vec2.x,test_vec2.y);
                    dx = (test_vec2.x - test_vec.x);
                    dy = (test_vec2.y - test_vec.y);
    
                    discriminant = get_discriminant(D,radius,dr);
                    if (discriminant>=0.0){
                        // ROS_WARN("NEW POINT? at %d",i);
                        intersections = get_points_intersection(D,dx,dy,radius,dr);

                        turtlelib::Vector2D laser_points;
                        turtlelib::Vector2D laser_points2;
                        laser_points.x = intersections.x1;
                        laser_points.y = intersections.y1;
                        laser_points2.x = intersections.x2;
                        laser_points2.y = intersections.y2;
                        laser_points = T_RO(laser_points);
                        laser_points2 = T_RO(laser_points2);
                        new_distance1 = get_distance(0,0,laser_points.x,laser_points.y);
                        new_distance2 = get_distance(0,0,laser_points2.x,laser_points2.y);

                        if (new_distance1>new_distance2)
                        {
                            new_distance1 = new_distance2;
                            points_compare_x = laser_points2.x;
                            points_compare_y = laser_points2.y;
                        }
                        else
                        {
                            new_distance1 = new_distance1;
                            points_compare_x = laser_points.x;
                            points_compare_y = laser_points.y;
                        }

                        if (new_distance1<get_distance(points_compare_x,points_compare_y,first_point.x,first_point.y))
                        {
                            // ROS_WARN("get_distance %f at %d",get_distance(points_compare_x,points_compare_y,second_point.x,second_point.y),i);
                            new_distance1 = max_scan_range+1;
                        }
                        distance_list.push_back(new_distance1);
                    }   
                    else
                    {
                        new_distance1 = max_scan_range+1;
                    }
                }

                double distance_to_use = max_scan_range+1;
                for (int k = 0;k<distance_list.size();k++)
                {
                    if (k==0){
                        distance_to_use = distance_list[0];
                    }
                    else if (k >=1 )
                    {
                        if(distance_list[k]<distance_to_use)
                        {
                            distance_to_use = distance_list[k];
                        }
                    }
                }

                turtlelib::Vector2D robot_in_robot;
                turtlelib::Vector2D robot_offset_in_robot;
                robot_in_robot.x = current_config.x;
                robot_in_robot.y = current_config.y;
                robot_offset_in_robot.x = current_config.x +cos(scan_angle+current_config.theta);
                robot_offset_in_robot.y = current_config.y +sin(scan_angle+current_config.theta);

                if (distance_to_use>= max_scan_range)
                {
                    double x1 = robot_in_robot.x;
                    double y1 = robot_in_robot.y;
                    double x2 = robot_offset_in_robot.x;
                    double y2 = robot_offset_in_robot.y ;
                    double d_12_x, d_34_x,d_12_1_x,d_34_1_x;
                    double d_12_y, d_34_y,d_12_1_y,d_34_1_y;
                    double px,py;
                    std::vector<wall_points> laser_wall;
                    laser_wall = create_walls();
                    std::vector<double> wall_distances;
                    for (int h = 0;h<4;h++)
                    {
                        // ROS_WARN("%d %d",i,h);   
                        turtlelib::Vector2D walls_p1, walls_p2;
                        double temp_dist;
                        walls_p1.x = laser_wall[h].x1;    
                        walls_p1.y = laser_wall[h].y1;   
                        walls_p2.x = laser_wall[h].x2;   
                        walls_p2.y = laser_wall[h].y2;        


                        d_12_x = get_D(x1,y1,x2,y2);
                        d_34_x = get_D(walls_p1.x,walls_p1.y,walls_p2.x,walls_p2.y);
                        d_12_1_x = get_D(x1,1.0,x2,1.0);
                        d_34_1_x = get_D(walls_p1.x,1.0,walls_p2.x,1.0);

                        d_12_y = get_D(x1,y1,x2,y2);
                        d_34_y = get_D(walls_p1.x,walls_p1.y,walls_p2.x,walls_p2.y);
                        d_12_1_y = get_D(y1,1.0,y2,1.0);
                        d_34_1_y = get_D(walls_p1.y,1.0,walls_p2.y,1.0);

                        px = get_D(d_12_x,d_12_1_x,d_34_x,d_34_1_x)/get_D(d_12_1_x,d_12_1_y,d_34_1_x,d_34_1_y);
                        py = get_D(d_12_y,d_12_1_y,d_34_y,d_34_1_y)/get_D(d_12_1_x,d_12_1_y,d_34_1_x,d_34_1_y);
                        turtlelib::Vector2D p_vect;
                        p_vect.x = px;
                        p_vect.y = py;
                        p_vect = T_RW(p_vect);
                        px = p_vect.x;
                        py = p_vect.y;

                        temp_dist = sqrt(px*px+py*py);

                        if (temp_dist<get_distance(px,py,first_point.x,first_point.y))
                        {
                            temp_dist = max_scan_range+1;
                        }

                        if (temp_dist<max_scan_range)
                        {
                            wall_distances.push_back(temp_dist);
                        }
                    }

                    for (int l = 0;l<wall_distances.size();l++)
                    {
                        // ROS_WARN("%f %d",wall_distances[l],i);
                        if(l==0){
                            distance_to_use = wall_distances[0];
                        }
                        
                        if(distance_to_use>wall_distances[l])
                        {
                            distance_to_use = wall_distances[l];
                        }                        
                    }
                }
                // ROS_WARN("actually use %f at %d",distance_to_use,i);
                if(distance_to_use > max_scan_range)
                {
                    distance_to_use = 0.0;
                }
                fake_laser.ranges[i] = distance_to_use + laser_noise(get_random());
                

            }
            fake_sensor_pub.publish(fake_laser);

        }

        /// \brief Get the wheel_command values and turn them into ticks
        ///
        /// \param sd - Wheel commands
        void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands &wheel_commands) 
        {
            wheel_Command = wheel_commands;
            //limit wheel commands
            if(wheel_Command.left_velocity > motor_cmd_max_upper){
                wheel_Command.left_velocity = motor_cmd_max_upper;
            }
            if(wheel_Command.right_velocity > motor_cmd_max_upper){
                wheel_Command.right_velocity = motor_cmd_max_upper;
            }
            if(wheel_Command.left_velocity < motor_cmd_max_lower){
                wheel_Command.left_velocity = motor_cmd_max_lower;
            }
            if(wheel_Command.right_velocity < motor_cmd_max_lower){
                wheel_Command.right_velocity = motor_cmd_max_lower;
            }
            
            left_tick = wheel_Command.left_velocity;
            right_tick = wheel_Command.right_velocity;
            
            std::normal_distribution<> left_vel_noise(0, robot_noise_stddev);
            std::normal_distribution<> right_vel_noise(0, robot_noise_stddev);

            left_noise = left_vel_noise(get_random());
            right_noise = right_vel_noise(get_random());
            
            if(turtlelib::almost_equal(left_tick,right_tick,0.01))
            {
                wheels_velocity.left_vel = (left_tick* motor_cmd_to_radsec); 
                wheels_velocity.right_vel = (right_tick * motor_cmd_to_radsec); 
            }
            else
            {
                wheels_velocity.left_vel = (left_tick* motor_cmd_to_radsec)+left_noise*(left_tick* motor_cmd_to_radsec); 
                wheels_velocity.right_vel = (right_tick * motor_cmd_to_radsec)+right_noise*(right_tick* motor_cmd_to_radsec); 
            }
             
            
        }

        /// \brief A timer that updates the simulation
        ///
        void main_loop(const ros::TimerEvent &)
         {
            std::uniform_real_distribution<> slip(slip_min, slip_max);
            // slip_noise = slip(get_random());
            // ROS_WARN("%f",slip_noise);
            slip_noise = slip(get_random());
            
            sensorData.left_encoder = (int) (((wheels_velocity.left_vel*(1/rate))+(wheel_angles2.left_angle))/encoder_ticks_to_rad);
            slip_noise = slip(get_random());
            sensorData.right_encoder = (int) (((wheels_velocity.right_vel*(1/rate))+(wheel_angles2.right_angle))/encoder_ticks_to_rad);
            // encoder_pub.publish(sensorData);

            wheel_angles.left_angle = (((wheels_velocity.left_vel*(1/rate))+wheel_angles.left_angle));
            wheel_angles.right_angle = (((wheels_velocity.right_vel*(1/rate))+wheel_angles.right_angle));
            wheel_angles2.left_angle = (((wheels_velocity.left_vel*(1/rate))+wheel_angles2.left_angle))+slip_noise*wheels_velocity.left_vel/rate;
            wheel_angles2.right_angle = (((wheels_velocity.right_vel*(1/rate))+wheel_angles2.right_angle))+slip_noise*wheels_velocity.right_vel/rate;
            // wheel_angles2.left_angle = (((wheels_velocity.left_vel*(1/rate))+wheel_angles2.left_angle));
            // wheel_angles2.right_angle = (((wheels_velocity.right_vel*(1/rate))+wheel_angles2.right_angle));
            current_config = DiffDrive.forward_Kinematics(wheel_angles,current_config);

            //update timestep and transforms
            timestep.data++;
            timestep_pub.publish(timestep);  
            // ROS_WARN("x: %f y:%f theta:%f",current_config.x,current_config.y,current_config.theta);
            
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.transform.translation.x = current_config.x;
            transformStamped.transform.translation.y = current_config.y;
            transformStamped.transform.translation.z = 0.0;
            theta = current_config.theta;
            q.setRPY(0, 0, theta);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            
            if (sim_flag == "sim"){
                encoder_pub.publish(sensorData);
                broadcaster.sendTransform(transformStamped);
            }
            check_for_collision();
            if (collision_flag == false){
                collision_x = current_config.x;
                collision_y = current_config.y;
                collision_theta = current_config.theta;
            }
            // broadcaster.sendTransform(transformStamped);
            //publish the markers'/cylinders' position 
            //Create marker array
            // marker.markers.resize(obstacle_array_size);
            // for (int i = 0;i<obstacle_array_size;i++)
            // {
            //     marker.markers[i].header.stamp = ros::Time();
            //     marker.markers[i].header.frame_id = "world";
            //     shape = visualization_msgs::Marker::CYLINDER;
            //     marker.markers[i].type = shape;
            //     marker.markers[i].ns = "obstacles";
            //     marker.markers[i].action = visualization_msgs::Marker::ADD;
            //     marker.markers[i].id = i;

            //     marker.markers[i].pose.position.x = cylinder_marker_x[i];
            //     marker.markers[i].pose.position.y = cylinder_marker_y[i];
            //     marker.markers[i].pose.position.z = 0.125;
            //     marker.markers[i].pose.orientation.x = 0.0;
            //     marker.markers[i].pose.orientation.y = 0.0;
            //     marker.markers[i].pose.orientation.z = 0.0;
            //     marker.markers[i].pose.orientation.w = 1.0;

            //     marker.markers[i].scale.x = (2*radius);
            //     marker.markers[i].scale.y = (2*radius);
            //     marker.markers[i].scale.z = 0.25;
                
            //     marker.markers[i].color.r = 1.0;
            //     marker.markers[i].color.g = 0.0;
            //     marker.markers[i].color.b = 0.0;
            //     marker.markers[i].color.a = 1.0;
            // }
            // marker_pub.publish(marker);

            // current_path.header.stamp = ros::Time::now();
            // current_path.header.frame_id = "world";
            // path_pose.header.stamp = ros::Time::now();
            // path_pose.header.frame_id = "red-base_footprint";
            // path_pose.pose.position.x = current_config.x;
            // path_pose.pose.position.y = current_config.y;
            // path_pose.pose.orientation.x = q.x();
            // path_pose.pose.orientation.y = q.y();
            // path_pose.pose.orientation.z = q.z();
            // path_pose.pose.orientation.w = q.w();
            // current_path.poses.push_back(path_pose);
            // path_pub.publish(current_path);
         }
        void fake_loop(const ros::TimerEvent &)
        {
            current_path.header.stamp = ros::Time::now();
            current_path.header.frame_id = "world";
            path_pose.header.stamp = ros::Time::now();
            path_pose.header.frame_id = "red-base_footprint";
            path_pose.pose.position.x = current_config.x;
            path_pose.pose.position.y = current_config.y;
            path_pose.pose.orientation.x = q.x();
            path_pose.pose.orientation.y = q.y();
            path_pose.pose.orientation.z = q.z();
            path_pose.pose.orientation.w = q.w();
            current_path.poses.push_back(path_pose);
            path_pub.publish(current_path);
            make_fake_obstacles();
            make_fake_laser();
        }
    
    private:
    //create private variables
        ros::NodeHandle nh;
        ros::Publisher timestep_pub;
        ros::Publisher marker_pub;
        ros::Publisher walls_pub;
        ros::Publisher encoder_pub;
        ros::Publisher path_pub;
        ros::Subscriber wheel_cmd_sub;
        ros::Publisher fake_sensor_pub;

        ros::Timer timer;
        ros::Timer timer_fake;
        double rate;
        std_msgs::UInt64 timestep;
        ros::ServiceServer reset_service;
        ros::ServiceServer teleport_service;
        sensor_msgs::JointState joint_state;
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped transformStamped;
        tf2::Quaternion q;
        geometry_msgs::Pose current_Pose;
        double theta;
        visualization_msgs::MarkerArray marker;
        visualization_msgs::MarkerArray x_walls;
    
        
        int obstacle_array_size;
        uint32_t shape;
        int num_obstacles;
        std::vector<double> cylinders_x_coord;
        std::vector<double> cylinders_y_coord;
        std::vector<double> cylinder_marker_x;
        std::vector<double> cylinder_marker_y;
        std::vector<double> robot_coords;
        double radius;
        double x_length;
        double y_length;
        
        int left_tick;
        int right_tick;
        // from DiffDrive
        turtlelib::phi_angles wheel_angles;
        turtlelib::phi_angles wheel_angles2;
        turtlelib::config current_config;
        turtlelib::speed wheels_velocity;
        turtlelib::speed slip_wheels_velocity;

        turtlelib::DiffDrive DiffDrive;
        double motor_cmd_to_radsec;
        double encoder_ticks_to_rad;
        nuturtlebot_msgs::SensorData sensorData;
        nuturtlebot_msgs::WheelCommands wheel_Command;
        std::vector<double> motor_cmd_max;
        double motor_cmd_max_lower;
        double motor_cmd_max_upper;

        int new_tick_left;
        int new_tick_right;
        int old_tick_left;
        int old_tick_right;
    
        double left_noise;
        double right_noise;
        double slip_min;
        double slip_max;
        double slip_noise;
        double obstacle_noise;
        double basic_sensor_variance;
        double distance;
        double max_range;
        visualization_msgs::MarkerArray fake_marker;
        ros::Publisher fake_marker_pub;
        nav_msgs::Path current_path;
        geometry_msgs::PoseStamped path_pose;
        std::string sim_flag;
        sensor_msgs::LaserScan fake_laser;
        double samples;
        double resolution;
        double min_angle;
        double max_angle;
        double scan_time;
        double min_scan_range;
        double max_scan_range;
        double robot_noise_mean;
        double robot_noise_stddev;
        double laser_variance;

        bool collision_flag;
        double collision_x;
        double collision_y;
        double collision_theta;
        double collision_radius;
        
};

/// \brief the main function that calls the class

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    Sim node;
    ros::spin();
    return 0;
}