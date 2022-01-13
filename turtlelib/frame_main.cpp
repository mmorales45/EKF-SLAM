#include<iosfwd>
#include <cmath>
#include "rigid2d.hpp"
#include <iostream>

int main(){
    turtlelib::Vector2D T;
    // turtlelib::Transform2D a = turtlelib::Transform2D();
    // std::cout << a;
    T.x = 1.0;
    T.y = 2.0;
    std::cout << T;
    std::cout << "Enter vector v_b:";
    std::cin >>T;
    std::cout << T;

}