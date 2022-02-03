#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>
#include<cmath>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <typeinfo>


namespace turtlelib
{

    DiffDrive::DiffDrive()
    {
        configuration.x = 0;
        configuration.y = 0;
        configuration.theta = 0;
        phi__.left_angle = 0;
        phi__.right_angle = 0;
        phi_dot.left_vel = 0;
        phi_dot.right_vel = 0;
    }

    DiffDrive::DiffDrive(config config_,phi_angles phi_input,speed phidot_input)
    {
        configuration.x = config_.x;
        configuration.y = config_.y;
        configuration.theta = config_.theta;
        phi__.left_angle = phi_input.left_angle;
        phi__.right_angle = phi_input.right_angle;
        phi_dot.left_vel = phidot_input.left_vel;
        phi_dot.right_vel = phidot_input.right_vel;
    }

    DiffDrive::DiffDrive(config config_)
    {
        configuration.x = config_.x;
        configuration.y = config_.y;
        configuration.theta = config_.theta;
    }

    speed DiffDrive::inverse_Kinematics(Twist2D twist)
    {
        if(twist.y_dot != 0.0)
        {
            throw std::logic_error("Does not compute!");
        }
        speed rates; 
        rates.left_vel = (-body_radius*twist.theta_dot + twist.x_dot)/wheel_radius;
        rates.right_vel = (body_radius*twist.theta_dot + twist.x_dot)/wheel_radius;
        return rates;
    }

    Twist2D DiffDrive::Twist_from_wheelVel(speed new_vel){
        Twist2D twist;
        twist.theta_dot = (wheel_radius/(2*body_radius))*(-new_vel.left_vel+new_vel.right_vel);
        twist.x_dot = (wheel_radius/2)*(new_vel.left_vel+new_vel.right_vel);
        twist.y_dot = 0.0;
        return twist;
    }

    phi_angles DiffDrive::angles_From_Rate(phi_angles old_angels, speed wheel_vel){
        phi_angles new_angles;
        new_angles.left_angle = old_angels.left_angle + wheel_vel.left_vel * 1;
        new_angles.right_angle =  old_angels.right_angle + wheel_vel.right_vel * 1;

        return new_angles;
    }

    Twist2D DiffDrive::Twist_from_wheelRates(phi_angles new_angles){
        Twist2D twist;
        // phi_angles angle_diff;
        phi_dot.left_vel = (new_angles.left_angle - phi__.left_angle);
        phi_dot.right_vel = (new_angles.right_angle - phi__.right_angle);

        twist.theta_dot = (wheel_radius/(2*body_radius))*(-phi_dot.left_vel+phi_dot.right_vel);
        twist.x_dot = (wheel_radius/2)*(phi_dot.left_vel+phi_dot.right_vel);
        twist.y_dot = 0.0;
        return twist;
    }
    // config DiffDrive::new_configuration(phi_angles angles, speed rates, Twist2D twist)


    // config DiffDrive::new_configuration(phi_angles old_angles,phi_angles new_angles, speed rates)
    config DiffDrive::forward_Kinematics(phi_angles new_angles)
    {
        Transform2D transform, Twb,TbbPrime, TwbPrime;
        Vector2D trans, updated_trans;
        double rot;
        speed wheel_speeds;
        Twist2D twist;

        phi_angles new_wheel_angles, angle_diff;
        
        phi_dot.left_vel = (new_angles.left_angle - phi__.left_angle);
        phi_dot.right_vel = (new_angles.right_angle - phi__.right_angle);

        phi__.left_angle = new_angles.left_angle;
        phi__.right_angle = new_angles.right_angle;

        twist.theta_dot = (wheel_radius/(2*body_radius))*(-phi_dot.left_vel+phi_dot.right_vel);
        twist.x_dot = (wheel_radius/2)*(phi_dot.left_vel+phi_dot.right_vel);
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

    config DiffDrive::forward_Kinematics(Twist2D twist)
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