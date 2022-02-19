/// \file
/// \brief This file creates custom services that affect the turtlebot as well as create obstacles in RVIZ.
///
/// PARAMETERS:
///     rate (double): Frequency at which the main loop runs
///     cylinders_x_coord (std::vector<double>): The x coordaintes of the three red cylinders
///     cylinders_y_coord (std::vector<double>): The y coordaintes of the three red cylinders
///     robot (std::vector<double>): vector/array of the robot's iniital coordinates in x,y,and theta
///     radius (double): the radius of the red cylinders
/// PUBLISHES:
///     timestep_pub (std_msgs/UInt64): Tracks the simulation's current timestep
///     joint_state_pub (sensor_msgs/JointState): Publishes joint positions to red/joint_states
///     marker_pub (visualization_msgs/MarkerArray): Creates obstacles as red cylinders in RVIZ
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


struct intersection_points {
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
            turtlelib::Vector2D new_trans = T_WR.translation();
            double new_angle = T_WR.rotation();

            
            fake_marker.markers.resize(obstacle_array_size);
            for (int i = 0;i<obstacle_array_size;i++)
            {
                std::normal_distribution<> fake_obstacle_noise(0, basic_sensor_variance);
                obstacle_noise = fake_obstacle_noise(get_random());
                turtlelib::Vector2D obs_trans;
                obs_trans.x = cylinder_marker_x[i];
                obs_trans.y = cylinder_marker_y[i];
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

                fake_marker.markers[i].pose.position.x = new_obs.x+obstacle_noise;
                fake_marker.markers[i].pose.position.y = new_obs.y+obstacle_noise;
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

        double get_distance(double x1, double y1, double x2, double y2)
        {
            double calc_distance;
            calc_distance = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
            return calc_distance;
        }
        
        void check_for_collision()
        {
            double collision_distance;
            for (int i = 0;i<obstacle_array_size;i++)
            {
                collision_distance = get_distance(current_config.x,current_config.y,cylinder_marker_x[i],cylinder_marker_y[i]);
                if ((collision_distance<(collision_radius+radius)) && (collision_flag==false))
                {   
                    collision_flag=true;

                    ROS_WARN("COLLISION!");
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
        double get_dr(double dx, double dy)
        {
            double dr;
            dr = sqrt(pow(dx,2)+pow(dy,2));
            return dr;
        }

        double get_D(double x1,double y1,double x2, double y2)
        {
            double D;
            D = x1*y2-x2*y1;
            return D;
        }

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

        intersection_points get_points_intersection(double D, double dx, double dy,double r,double dr)
        {
            intersection_points points_of_interest;
            // double x1,y1,x2,y2;
            points_of_interest.x1 = (D*dy+get_sgn(dy)*dx*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));
            points_of_interest.x2 = (D*dy-get_sgn(dy)*dx*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));
            points_of_interest.y1 = (-D*dy+get_sgn(dx)*fabs(dy)*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));
            points_of_interest.y2 = (-D*dy-get_sgn(dx)*fabs(dy)*sqrt(pow(r,2)*pow(dr,2)-pow(D,2)))/(pow(dr,2));

            return points_of_interest;
        }

        double get_discriminant(double D, double r, double dr)
        {
            double discriminant;
            discriminant = pow(r,2)*pow(dr,2)-pow(D,2);
            return discriminant;
        }


        void make_fake_laser()
        {
            turtlelib::Vector2D min_range_vector;
            turtlelib::Vector2D max_range_vector;
            double scan_angle;
            double D, discriminant;
            double dx,dy;
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
            obstacle_robot_obs.resize(3);
            robot_obs_robot.resize(3);
            // for (int i = 0;i<obstacle_array_size;i++)
            // {
            //     turtlelib::Vector2D obstacle_translational;
            //     obstacle_translational.x = cylinder_marker_x[i];
            //     obstacle_translational.y = cylinder_marker_y[i];
            //     turtlelib::Transform2D T_WO;
            //     T_WO = turtlelib::Transform2D(obstacle_translational);
            //     turtlelib::Transform2D T_OW;
            //     T_OW = T_WO.inv();
            //     turtlelib::Transform2D T_OR;
            //     T_OR = T_OW*T_WR;
            //     turtlelib::Transform2D T_RO;
            //     T_RO = T_OR.inv();
            //     turtlelib::Vector2D new_obs = T_OR.translation();
            //     obstacle_robot_obs[i] = new_obs;
            //     // ROS_WARN("%f at %d obs",obstacle_robot_obs[i].x, i);
            //     // red_trans_in_obstacle = T_OR(red_trans);
            //     // robot_obs_robot[i] = red_trans_in_obstacle;
            //     // ROS_WARN("%f at %d ror",robot_obs_robot[i].x, i);

            // }
            fake_laser.header.frame_id = "red-base_scan";
            fake_laser.header.stamp = ros::Time::now();
            fake_laser.angle_min = min_angle;
            fake_laser.angle_max = max_angle;
            fake_laser.angle_increment = resolution;
            fake_laser.time_increment = 0.5/1000000000.0;
            fake_laser.scan_time = scan_time/1000000000.0;
            fake_laser.range_min = min_scan_range;
            fake_laser.range_max = max_scan_range;
            fake_laser.ranges.resize(samples);
            // fake_laser.ranges[i] = 1.0;
            for (int i = 0;i<samples;i++)
            {
                double new_distance1;
                double new_distance2;
                // fake_laser.ranges[i] = 1.0;
                scan_angle = turtlelib::deg2rad(i);
                min_range_vector.x = min_scan_range*cos(scan_angle);
                min_range_vector.y = min_scan_range*sin(scan_angle);
                max_range_vector.x = max_scan_range*cos(scan_angle);
                max_range_vector.y = max_scan_range*sin(scan_angle);
                for (int j = 0;j<obstacle_array_size;j++)
                {
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
                    // ROS_WARN("%f at %d obs",obstacle_robot_obs[i].x, i);
                    // red_trans_in_obstacle = T_OR(red_trans);
                    // robot_obs_robot[i] = red_trans_in_obstacle;
                    // ROS_WARN("%f at %d ror",robot_obs_robot[i].x, i);


                    new_distance1 = 0;
                    new_distance2 = 0;
                    turtlelib::Vector2D first_point = T_OR(min_range_vector)+obstacle_robot_obs[j];
                    turtlelib::Vector2D second_point = T_OR(max_range_vector)+obstacle_robot_obs[j];
                    D = get_D(first_point.x,first_point.y,second_point.x,second_point.y);
                    dx = (second_point.x - first_point.x);
                    dy = (second_point.y - first_point.y);
                    dr = get_dr(dx,dy);
                    discriminant = get_discriminant(D,radius,dr);
                    ROS_WARN("%f at %d",discriminant, i);
                    if (discriminant>=0.0){
                        ROS_WARN("NEW POINT?");
                        intersections = get_points_intersection(D,dx,dy,radius,dr);
                        new_distance1 = get_distance(first_point.x,first_point.y,intersections.x1,intersections.y1);
                        // new_distance2 = get_distance(first_point.x,first_point.y,intersections.x2,intersections.y2);

                        // fake_laser.ranges[i] = std::max(new_distance1+0.12,new_distance2+0.12);
                    }   
                

                }
                
                fake_laser.ranges[i] = std::min(new_distance1,100.0);
                

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
            
            std::normal_distribution<> left_vel_noise(0, .02);
            std::normal_distribution<> right_vel_noise(0, .02);
            // std::normal_distribution<> slip(0.5, slip_max);
            // std::normal_distribution<> fake_obstacle_noise(0, basic_sensor_variance);
            left_noise = left_vel_noise(get_random());
            right_noise = right_vel_noise(get_random());
            // slip_noise = slip(get_random());
            
            // wheels_velocity.left_vel = (left_tick* motor_cmd_to_radsec); 
            // wheels_velocity.right_vel = (right_tick * motor_cmd_to_radsec); 
            wheels_velocity.left_vel = (left_tick* motor_cmd_to_radsec)+left_noise*(left_tick* motor_cmd_to_radsec); 
            wheels_velocity.right_vel = (right_tick * motor_cmd_to_radsec)+right_noise*(right_tick* motor_cmd_to_radsec); 
            // slip_wheels_velocity.left_vel = wheels_velocity.left_vel + left_noise*(left_tick* motor_cmd_to_radsec)
            // slip_wheels_velocity.right_vel = wheels_velocity.right_vel + right_noise*(right_tick* motor_cmd_to_radsec); 
            
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
         }
        void fake_loop(const ros::TimerEvent &)
        {
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