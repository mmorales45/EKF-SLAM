#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include<cmath>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <typeinfo>


namespace turtlelib
{

    diff_drive::diff_drive()
    {
        configuration.x = 0;
        configuration.y = 0;
        configuration.theta = 0;
        phi__.phi_left = 0;
        phi__.phi_right = 0;
        phi_dot.phi_left = 0;
        phi_dot.phi_right = 0;
    }

    diff_drive::diff_drive(config config_,phi_angles phi_input,speed phidot_input)
    {
        configuration.x = config_.x;
        configuration.y = config_.y;
        configuration.theta = config_.theta;
        phi__.phi_left = phi_input.phi_left;
        phi__.phi_right = phi_input.phi_right;
        phi_dot.phi_left = phidot_input.phi_left;
        phi_dot.phi_right = phidot_input.phi_right;
    }

    diff_drive::diff_drive(config config_)
    {
        configuration.x = config_.x;
        configuration.y = config_.y;
        configuration.theta = config_.theta;
    }

    speed diff_drive::inverse_Kinematics(Twist2D twist)
    {
        if(twist.y_dot != 0.0)
        {
            throw std::logic_error("Does not compute!");
        }
        speed rates; 
        rates.phi_left = (-body_radius*twist.theta_dot + twist.x_dot)/wheel_radius;
        rates.phi_right = (body_radius*twist.theta_dot + twist.x_dot)/wheel_radius;
        return rates;
    }

    Twist2D diff_drive::Twist_from_wheelVel(speed new_vel){
        Twist2D twist;
        twist.theta_dot = (wheel_radius/(2*body_radius))*(-new_vel.phi_left+new_vel.phi_right);
        twist.x_dot = (wheel_radius/2)*(new_vel.phi_left+new_vel.phi_right);
        twist.y_dot = 0.0;
        return twist;
    }

    Twist2D diff_drive::Twist_from_wheelRates(phi_angles new_angles){
        Twist2D twist;
        // phi_angles angle_diff;
        phi_dot.phi_left = (new_angles.phi_left - phi__.phi_left);
        phi_dot.phi_right = (new_angles.phi_right - phi__.phi_right);

        twist.theta_dot = (wheel_radius/(2*body_radius))*(-phi_dot.phi_left+phi_dot.phi_right);
        twist.x_dot = (wheel_radius/2)*(phi_dot.phi_left+phi_dot.phi_right);
        twist.y_dot = 0.0;
        return twist;
    }
    // config diff_drive::new_configuration(phi_angles angles, speed rates, Twist2D twist)


    // config diff_drive::new_configuration(phi_angles old_angles,phi_angles new_angles, speed rates)
    config diff_drive::forward_Kinematics(phi_angles new_angles)
    {
        Transform2D transform, Twb,TbbPrime, TwbPrime;
        Vector2D trans, updated_trans;
        double rot;
        speed wheel_speeds;
        Twist2D twist;

        phi_angles new_wheel_angles, angle_diff;
        
        phi_dot.phi_left = (new_angles.phi_left - phi__.phi_left);
        phi_dot.phi_right = (new_angles.phi_right - phi__.phi_right);

        phi__.phi_left = new_angles.phi_left;
        phi__.phi_right = new_angles.phi_right;

        twist.theta_dot = (wheel_radius/(2*body_radius))*(-phi_dot.phi_left+phi_dot.phi_right);
        twist.x_dot = (wheel_radius/2)*(phi_dot.phi_left+phi_dot.phi_right);
        twist.y_dot = 0.0;

        trans.x = configuration.x;
        trans.y = configuration.y;
        Twb = Transform2D(trans,configuration.theta);
        TbbPrime = integrate_twist(twist);
        TwbPrime = Twb*TbbPrime;
        updated_trans = TwbPrime.translation();
        rot = normalize_angle(TwbPrime.rotation());
        
        configuration.x = updated_trans.x;
        configuration.y = updated_trans.y;
        configuration.theta = rot;

        return configuration;
    }

    config diff_drive::forward_Kinematics(Twist2D twist)
    {
        Transform2D transform, Twb,TbbPrime, TwbPrime;
        Vector2D trans, updated_trans;
        double rot;
        speed wheel_speeds;

        trans.x = configuration.x;
        trans.y = configuration.y;
        Twb = Transform2D(trans,configuration.theta);
        TbbPrime = integrate_twist(twist);
        TwbPrime = Twb*TbbPrime;
        updated_trans = TwbPrime.translation();
        rot = normalize_angle(TwbPrime.rotation());
        
        configuration.x = updated_trans.x;
        configuration.y = updated_trans.y;
        configuration.theta = rot;

        return configuration;
    }



}