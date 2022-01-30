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
        
        Twist2D Twist_from_wheelRates(speed phi);

        speed get_phi_rates(Twist2D twist);

        phi_angles new_configuration(phi_angles updated_angles, speed rates, Twist2D twist);

        phi_angles new_configuration(phi_angles old_angles,phi_angles new_angles, speed rates);

        phi_angles new_angles(speed angle_rate, phi_angles old_angles);


    private:
        // speed rates;
        config configuration;
        // phi_angles 
    };


}
#endif