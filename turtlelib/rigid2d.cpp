#include "rigid2d/rigid2d.hpp"

turtlelib::Transform2D::Transform2D()
{
    x = 0;
    y = 0;
    theta = 0;
} 

turtlelib::Transform2D::Transform2D(Vector2D trans)
{
    x = trans.x;
    y = trans.y;
    theta = 0;
}

turtlelib::Transform2D::Transform2D(double radians)
{
    x = 0;
    y = 0;
    theta = radians;
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    x = trans.x;
    y = trans.y;
    theta = radians;
}

