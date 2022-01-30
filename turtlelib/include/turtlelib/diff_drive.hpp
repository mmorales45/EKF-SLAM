#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

/// \file
/// \brief Modeling the kinematics of a differential drive robot


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>
#include <turtlelib/rigid2d.hpp>

namespace turtlelib
{
    constexpr double wheel_radius=0.033;
    constexpr double body_radius=0.08;

    struct speed
    {
        double phi_left = 0.0;
        double phi_right = 0.0;
    };

    struct config
    {
        double theta = 0.0;
        double x = 0.0;
        double y = 0.0;

    };

    struct phi_angles
    {
        double phi_left = 0.0;
        double phi_right = 0.0;
    };


    class diff_drive
    {
    public:
        diff_drive();

        diff_drive(config config_,phi_angles phi_input,speed phidot_input);
        
        Twist2D Twist_from_wheelRates(phi_angles new_angles);

        speed inverse_Kinematics(Twist2D twist);

        config forward_Kinematics(phi_angles new_angles);

        config forward_Kinematics(Twist2D twist);


    private:
        speed phi_dot;
        config configuration;
        phi_angles phi__;
    };


}
#endif