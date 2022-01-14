#include<iosfwd>
#include <cmath>
#include "rigid2d.hpp"
#include <iostream>

int main(){
    turtlelib::Vector2D T;
    turtlelib::Twist2D twist;
    turtlelib::Transform2D Tab = turtlelib::Transform2D();
    turtlelib::Transform2D Tba;
    turtlelib::Transform2D Tbc = turtlelib::Transform2D();
    turtlelib::Transform2D Tcb;
    turtlelib::Transform2D Tac;
    turtlelib::Transform2D Tca;
    

    // std::cout << "Enter twist V_b: \n";
    // std::cin >> twist;
    // std::cout << twist;



    std::cout << "Enter transform T_{a,b}: \n";
    std::cin >> Tab;

    std::cout << "Enter transform T_{b,c}: \n";
    std::cin >> Tbc;
    
    Tba = Tab.inv();
    Tac = Tab*Tbc;
    Tcb = Tbc.inv();
    Tca = Tac.inv();

    std::cout << "T_{a,b}: " <<Tab;
    std::cout << "T_{b,a}: " <<Tba;
    std::cout << "T_{b,c}: " <<Tbc;
    std::cout << "T_{c,b}: " <<Tcb;
    std::cout << "T_{a,c}: " <<Tac;
    std::cout << "T_{c,a}: " <<Tca;

    std::cout << "Enter vector v_b: \n";
    std::cin >>T;
    std::cout << T;
    
}