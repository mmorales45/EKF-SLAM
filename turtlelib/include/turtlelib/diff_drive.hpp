#ifndef DiffDrive_INCLUDE_GUARD_HPP
#define DiffDrive_INCLUDE_GUARD_HPP

/// \file
/// \brief Modeling the kinematics of a differential drive robot


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include <turtlelib/rigid2d.hpp>

namespace turtlelib
{
    /// \brief Radius of the wheel of the robot
    constexpr double wheel_radius=0.033;

    /// \brief Length of the center of the robot to the wheel
    constexpr double body_radius=0.08;

    /// \brief A vector made up of the left and right wheel speeds
    struct speed
    {
        /// \brief the left wheel speed
        double left_vel = 0.0;

        /// \brief the right wheel speed
        double right_vel = 0.0;
    };

    /// \brief A vector that holds the elements of the current theta,x and y positions of the robot.
    struct config
    {
        /// \brief the current theta of the robot.
        double theta = 0.0;

        /// \brief the current x of the robot.
        double x = 0.0;

        /// \brief the current y of the robot.
        double y = 0.0;

    };

    /// \brief A vector made up of the left and right wheel angles
    struct phi_angles
    {
        /// \brief the left wheel angle
        double left_angle = 0.0;

        /// \brief the right wheel angle
        double right_angle = 0.0;
    };

    /// \brief Calculate Kinemmatics of a robot
    class DiffDrive
    {
    public:
        /// \brief Create an object with configuration, angles and speeds as 0
        DiffDrive();

        /// \brief Create an object with configuration, angles and angle speeds
        /// \param config_ - x,y,theta of the robot
        /// \param phi_input - angles of the left and right wheels
        /// \param phidot_input - speeds of the left and right wheels
        DiffDrive(config config_,phi_angles phi_input,speed phidot_input);

        DiffDrive(config config_);
        
        /// \brief Calculate a twist from new wheel angles
        /// \param new_angles - The new angles of the wheel
        /// \return a twist composing of x_dot,y_dot and theta_dot
        Twist2D Twist_from_wheelRates(phi_angles new_angles);

        Twist2D Twist_from_wheelVel(speed new_vel);
        phi_angles angles_From_Rate(phi_angles old_angels, speed wheel_vel);

        /// \brief Calculate the speed using inverse kinematics
        /// \param twist - the x_dot,y_dot and theta_dot
        /// \return - the speed of the robot
        speed inverse_Kinematics(Twist2D twist);

        /// \brief Calculate the current x,y,z using forward kinematics
        /// \param new_angles - the new angles of the robot
        /// \return - the updated configuration 
        config forward_Kinematics(phi_angles new_angles);


        config forward_Kinematics(phi_angles new_angles,config new_config);

        /// \brief Calculate the current x,y,z using forward kinematics using twist
        /// \param twist - the x_dot,y_dot and theta_dot
        /// \return - the updated configuration 
        config forward_Kinematics(Twist2D twist);

        


    private:
        speed phi_dot;
        config configuration;
        phi_angles phi__;
    };


}
#endif