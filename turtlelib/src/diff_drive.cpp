#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include<cmath>


namespace turtlelib
{
    // diff_drive::diff_drive()
    // {
    //     double phi1 = 0.0;
    //     double phi2 = 0.0;
    //     double x = 0.0;
    //     double y = 0.0;
    //     double theta = 0.0;
    // }

    speed diff_drive::get_phi_rates(Twist2D twist)
    {
        speed rates; 
        rates.phi_left = (-body_radius*twist.theta_dot + twist.x_dot)/wheel_radius;
        rates.phi_right = (body_radius*twist.theta_dot + twist.x_dot)/wheel_radius;
        return rates;
    }

    phi_angles diff_drive::new_angles(speed angle_rate, phi_angles old_angles)
    {
        phi_angles updated_angles;
        updated_angles.phi_left = old_angles.phi_left + angle_rate.phi_left;
        updated_angles.phi_right = old_angles.phi_right + angle_rate.phi_right;
        return updated_angles;
    }

    phi_angles diff_drive::new_configuration(phi_angles updated_angles, speed rates, Twist2D twist)
    {
        Transform2D transform, Twb,TbbPrime, TwbPrime;
        Vector2D trans;
        speed wheel_speeds;
        phi_angles new_wheel_angles;
        trans.x = configuration.x;
        trans.y = configuration.y;
        Twb = Transform2D(trans,configuration.theta);
        TbbPrime = integrate_twist(twist);
        TwbPrime = Twb*TbbPrime;

        wheel_speeds = diff_drive::get_phi_rates(twist);

        new_wheel_angles = diff_drive::new_angles(rates, updated_angles);

        return new_wheel_angles;



    }



}