#include "rigid2d/rigid2d.hpp"
#include <cmath>

turtlelib::Transform2D::Transform2D()
{
    translational_component.x = 0;
    translational_component.y = 0;
    angular_displacement = 0;
} 

turtlelib::Transform2D::Transform2D(Vector2D trans)
{
    translational_component.x = trans.x;
    translational_component.y = trans.y;
    angular_displacement = 0;
}

turtlelib::Transform2D::Transform2D(double radians)
{
    translational_component.x = 0;
    translational_component.y = 0;
    angular_displacement = radians;
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    translational_component.x = trans.x;
    translational_component.y = trans.y;
    angular_displacement = radians;
}

turtlelib::Transform2D::inv()
{
    double inverse_theta;
    Vector2D inverse_translation;
    Transform2D inverse_transform;

    inverse_theta = -angular_displacement;
    inverse_translation.x = (-translational_component.x *cos(angular_displacement)) - (translational_component.y * sin(angular_displacement));
    inverse_translation.x = (-translational_component.y *cos(angular_displacement)) + (translational_component.x * sin(angular_displacement));

    inverse_transform = Transform2D(Vector2D inverse_translation, double inverse_theta);
    return inverse_transform;
}

turtlelib::Transform2D::translation() const
{
    return translational_component;
}

turtlelib::Transform2D::rotation() const
{
    return angular_displacement;
}


