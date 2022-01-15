#include<iosfwd>
#include <cmath>
#include "rigid2d.hpp"
#include <iostream>

int main(){
    turtlelib::Vector2D v_b;
    turtlelib::Vector2D v_a;
    turtlelib::Vector2D v_c;
    turtlelib::Vector2D v_bhat;
    turtlelib::Transform2D Tab;
    turtlelib::Transform2D Tba;
    turtlelib::Transform2D Tbc;
    turtlelib::Transform2D Tcb;
    turtlelib::Transform2D Tac;
    turtlelib::Transform2D Tca;


    turtlelib::Twist2D V_a;
    turtlelib::Twist2D V_b;
    turtlelib::Twist2D V_c;
    

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
    std::cin >>v_b;
    v_a = Tab(v_b);
    v_c = Tcb(v_b);
    v_bhat = Normalize(v_b);

    std::cout <<"v_bhat" << v_bhat;
    std::cout <<"v_a " <<v_a;
    std::cout <<"v_b " <<v_b;
    std::cout <<"v_c " <<v_c;

    std::cout <<"Enter twist V_b: \n";
    std::cin >> V_b;

    V_a = Tab(V_b);
    V_c = Tcb(V_b);

    std::cout << "V_a "<<V_a;
    std::cout << "V_b "<<V_b;
    std::cout << "V_c "<<V_c;
}