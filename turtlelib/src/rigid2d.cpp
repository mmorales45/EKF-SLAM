/// \file
/// \brief Defines functions and operators that were declared in rigid2D.hpp
///

#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <string>

namespace turtlelib{

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) 
    {
        return os << "["<<v.x << " "<< v.y<<"]"<<"\n";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) 
    {
        char c1 = is.peek();
        
        if (c1 == '['){ 
            is.get();
            is >> v.x;
            is >> v.y;
            is.get();
            is.get();
        }
        else{
            is >> v.x >> v.y;
        }
        return is;
    }

    Vector2D Normalize(Vector2D vector)
    {
        Vector2D v_hat;
        double magnitude = sqrt(pow(vector.x,2)+ pow(vector.y,2));
        v_hat.x = vector.x/magnitude;
        v_hat.y = vector.y/magnitude;
        return v_hat; 
    }

    Transform2D::Transform2D()
    {
        translational_component.x = 0;
        translational_component.y = 0;
        angular_displacement = 0;

    } 
    Transform2D::Transform2D(Vector2D trans) 
    {
        translational_component.x = trans.x;
        translational_component.y = trans.y;
        angular_displacement = 0;
    }

    Transform2D::Transform2D(double radians)
    {
        translational_component.x = 0;
        translational_component.y = 0;
        angular_displacement = radians;
    }

    Transform2D::Transform2D(Vector2D trans, double radians)
    {
        translational_component.x = trans.x;
        translational_component.y = trans.y;
        angular_displacement = radians;
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        Vector2D transformed_vector;
        transformed_vector.x = v.x*cos(angular_displacement) - (v.y*sin(angular_displacement)) + translational_component.x;
        transformed_vector.y = v.x*sin(angular_displacement) + (v.y*cos(angular_displacement)) + translational_component.y;

        return transformed_vector;
    }

    Twist2D Transform2D::operator()(Twist2D twist) const
    {
        Twist2D new_twist;
        new_twist.theta_dot = twist.theta_dot;
        new_twist.x_dot = (twist.theta_dot * translational_component.y)+(twist.x_dot*cos(angular_displacement))-(twist.y_dot*sin(angular_displacement));
        new_twist.y_dot = (-twist.theta_dot * translational_component.x)+(twist.x_dot*sin(angular_displacement))+(twist.y_dot*cos(angular_displacement));

        return new_twist;
    }


    Transform2D Transform2D::inv() const
    {
        double inverse_theta;
        Vector2D inverse_translation;
        Transform2D inverse_transform;
        
        inverse_theta = -angular_displacement;

        inverse_translation.x = (-translational_component.x *cos((angular_displacement))) - (translational_component.y * sin((angular_displacement)));
        inverse_translation.y = (-translational_component.y *cos((angular_displacement))) + (translational_component.x * sin((angular_displacement)));

        inverse_transform = Transform2D(inverse_translation, inverse_theta);
        return inverse_transform;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        translational_component.x = (rhs.translational_component.x)*cos(angular_displacement) - (rhs.translational_component.y)*sin(angular_displacement) + translational_component.x;
        translational_component.y = (rhs.translational_component.x)*sin(angular_displacement) + (rhs.translational_component.y)*cos(angular_displacement) + translational_component.y;
        angular_displacement = angular_displacement+rhs.angular_displacement;
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return translational_component;
    }

    double Transform2D::rotation() const
    {
        return angular_displacement;
    }



    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        Vector2D translational;
        double rotational;
        
        char c1 = is.peek();
        char str1[6] = "";
        char str2[4] = "";

        if (c1 == 'd'){
            is.get(str1,6);
            is >> rotational;
            is.get(str2,4);
            is >> translational.x;
            is.get(str2,4);
            is.get();
            is >> translational.y;
            is.get();
        }
        else {
            is >> rotational >> translational.x >> translational.y;
            is.get();
        }
        rotational = deg2rad(rotational);
        tf = Transform2D(translational,rotational);

        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        os <<"deg: "<< rad2deg(tf.angular_displacement)<< " x: "<<tf.translational_component.x << " y: " <<tf.translational_component.y << "\n";
        return os;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs *= rhs; // from hint 

        return lhs;
    }


    std::ostream & operator<<(std::ostream & os, const Twist2D & t)
    {
        os << "["<< t.theta_dot << " " << t.x_dot << " " << t.y_dot <<"]"<< "\n";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & t)
    {   
        char c1 = is.peek();
        
        if (c1 == '['){ 
            is.get();
            is >> t.theta_dot;
            is.get();
            is >> t.x_dot;
            is.get();
            is >> t.y_dot;
            is.get();
            is.get();

        }
        else {
            is >> t.theta_dot >> t.x_dot >> t.y_dot;
        }
        return is;
    }

}