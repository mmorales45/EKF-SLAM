#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include<cmath>


namespace turtlelib
{

    diff_drive::diff_drive()
    {
        configuration.x = 0;
        configuration.y = 0;
        configuration.theta = 0;
    }

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

    Twist2D diff_drive::Twist_from_wheelRates(speed phi){
        Twist2D twist;
        twist.theta_dot = (wheel_radius/(2*body_radius))*(phi.phi_right-phi.phi_left);
        twist.x_dot = (wheel_radius/2)*(phi.phi_right-phi.phi_left);
        twist.y_dot = 0;
        return twist;
    }

    phi_angles diff_drive::new_configuration(phi_angles angles, speed rates, Twist2D twist)
    {
        Transform2D transform, Twb,TbbPrime, TwbPrime;
        Vector2D trans, updated_trans;
        double rot;
        speed wheel_speeds;
        phi_angles new_wheel_angles;

        trans.x = configuration.x;
        trans.y = configuration.y;
        Twb = Transform2D(trans,configuration.theta);
        TbbPrime = integrate_twist(twist);
        TwbPrime = Twb*TbbPrime;
        updated_trans = TwbPrime.translation();
        rot = TwbPrime.rotation();
        
        configuration.x = updated_trans.x;
        configuration.y = updated_trans.y;
        configuration.theta = rot;


        wheel_speeds = diff_drive::get_phi_rates(twist);

        new_wheel_angles = diff_drive::new_angles(wheel_speeds, angles);

        return new_wheel_angles;
    }

    phi_angles diff_drive::new_configuration(phi_angles old_angles,phi_angles new_angles, speed rates)
    {
        Transform2D transform, Twb,TbbPrime, TwbPrime;
        Vector2D trans, updated_trans;
        double rot;
        speed wheel_speeds;
        Twist2D twist;

        phi_angles new_wheel_angles, angle_diff;
        angle_diff.phi_left = new_angles.phi_left - old_angles.phi_left;
        angle_diff.phi_right = new_angles.phi_right - old_angles.phi_right;
        twist.theta_dot = (wheel_radius/2)*(-body_radius*angle_diff.phi_left+body_radius*angle_diff.phi_right);
        twist.x_dot = (wheel_radius/2)*(angle_diff.phi_left+angle_diff.phi_right);
        twist.y_dot = 0.0;

        trans.x = configuration.x;
        trans.y = configuration.y;
        Twb = Transform2D(trans,configuration.theta);
        TbbPrime = integrate_twist(twist);
        TwbPrime = Twb*TbbPrime;
        updated_trans = TwbPrime.translation();
        rot = TwbPrime.rotation();
        
        configuration.x = updated_trans.x;
        configuration.y = updated_trans.y;
        configuration.theta = rot;


        wheel_speeds = diff_drive::get_phi_rates(twist);

        new_wheel_angles = diff_drive::new_angles(rates, old_angles);

        return new_wheel_angles;
    }



}