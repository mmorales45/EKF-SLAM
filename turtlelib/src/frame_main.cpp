/// \file
/// \brief Creates transforms, 2D Vectors, 2D twists from user input and transform them to different frames
///

#include<iosfwd>
#include <cmath>
#include <turtlelib/rigid2d.hpp>
#include <iostream>

int main(){
    //Have the user input 2 transforms
    turtlelib::Transform2D Tab;
    std::cout << "Enter transform T_{a,b}: \n";
    std::cin >> Tab;
    turtlelib::Transform2D Tbc;
    std::cout << "Enter transform T_{b,c}: \n";
    std::cin >> Tbc;
    //Compute other transforms
    turtlelib::Transform2D Tba;
    Tba = Tab.inv();
    turtlelib::Transform2D Tac;
    Tac = Tab*Tbc;
    turtlelib::Transform2D Tcb;
    Tcb = Tbc.inv();
    turtlelib::Transform2D Tca;
    Tca = Tac.inv();
    //Print out transforms
    std::cout << "T_{a,b}: " <<Tab;
    std::cout << "T_{b,a}: " <<Tba;
    std::cout << "T_{b,c}: " <<Tbc;
    std::cout << "T_{c,b}: " <<Tcb;
    std::cout << "T_{a,c}: " <<Tac;
    std::cout << "T_{c,a}: " <<Tca;

    //Have the user input a vector and compute vector in other frames
    std::cout << "Enter vector v_b: \n";
    turtlelib::Vector2D v_b;
    std::cin >>v_b;
    turtlelib::Vector2D v_a;
    v_a = Tab(v_b);
    turtlelib::Vector2D v_c;
    v_c = Tcb(v_b);
    //normalize vector v_b
    turtlelib::Vector2D v_bhat;
    v_bhat = NormalizeVector(v_b);
    //print out the vectors
    std::cout <<"v_bhat" << v_bhat;
    std::cout <<"v_a " <<v_a;
    std::cout <<"v_b " <<v_b;
    std::cout <<"v_c " <<v_c;
    
    //Have the user input a twist
    std::cout <<"Enter twist V_b: \n";
    turtlelib::Twist2D V_b;
    std::cin >> V_b;
    //Compute twist in other frames and print out
    turtlelib::Twist2D V_a;
    V_a = Tab(V_b);
    turtlelib::Twist2D V_c;
    V_c = Tcb(V_b);

    std::cout << "V_a "<<V_a;
    std::cout << "V_b "<<V_b;
    std::cout << "V_c "<<V_c;

    // double normal_angle = turtlelib::PI;
    // normal_angle = turtlelib::normalize_angle(-1*normal_angle);
    // std::cout << "Normalized Angle: "<<normal_angle<<std::endl;

    // double scalar = 2;
    // double scalar3 = 3;
    // turtlelib::Vector2D vect;
    // turtlelib::Vector2D vect3;
    // vect.x = 1.0;
    // vect.y = 1.0;
    // vect3.x = 1.0;
    // vect3.y = 1.5;

    // vect3 *= scalar3;
    // std::cout << vect*scalar << std::endl;

    // vect3 = vect3 * scalar3;
    // std::cout << vect3 << std::endl;

    // vect3 = scalar3 * vect3;
    // std::cout << vect3 << std::endl;

    // std::cout <<  vect3 << std::endl;

    // double prod = dot(vect, vect3);
    // std::cout << prod << std::endl;

    // double mag = magnitude(vect);

    // std::cout << mag << std::endl;

    // double angle1 = angle(vect, vect3);

    // std::cout<< angle1 << std::endl;

    // turtlelib::Twist2D twist;
    // twist.x_dot = 5.0;
    // twist.y_dot = 2.0;
    // twist.theta_dot = turtlelib::PI/2;

    // turtlelib::Transform2D tbb;
    // tbb = turtlelib::integrate_twist(twist);
    // double rot = tbb.rotation();
    // std::cout << tbb << std::endl;
    // std::cout << rot << std::endl;
}