/// \file
/// \brief Tests the functions and operators defined in rigid2D.cpp
/// use g++ -Wall -Wextra -g -std=c++17 -o tests tests.cpp rigid2d.cpp to run tests
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "turtlelib/rigid2d.hpp"
#include <string>       
#include <iostream>    
#include <sstream> 


TEST_CASE("Test Identity Transform", "[transform]") {
    turtlelib::Transform2D transform = turtlelib::Transform2D();
    turtlelib::Vector2D vec = transform.translation();
    double angle = transform.rotation();
    double x_ph = vec.x;
    double y_ph = vec.x;

    REQUIRE( angle == 0 );
    REQUIRE( x_ph == 0 );
    REQUIRE( y_ph == 0 );
}

TEST_CASE("Test Translational Only Transform", "[transform]") {
    turtlelib::Vector2D vector;
    vector.x = 2.0;
    vector.y = 3.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(vector);
    turtlelib::Vector2D vec = transform.translation();
    double angle = transform.rotation();
    double x_ph = vec.x;
    double y_ph = vec.y;

    REQUIRE( angle == 0 );
    REQUIRE( x_ph == 2.0 );
    REQUIRE( y_ph == 3.0 );
}

TEST_CASE("Test Rotational Only Transform", "[transform]") {
    double angle_init = 90.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(angle_init);
    turtlelib::Vector2D vec = transform.translation();
    double angle = transform.rotation();
    double x_ph = vec.x;
    double y_ph = vec.y;

    REQUIRE( angle == 90.0 );
    REQUIRE( x_ph == 0.0 );
    REQUIRE( y_ph == 0.0 );
}

TEST_CASE("Test Both Translational and Rotational Transform", "[transform]") {
    double angle_init = 90.0;
    turtlelib::Vector2D vector;
    vector.x = 2.0;
    vector.y = 3.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(vector,angle_init);
    turtlelib::Vector2D vec = transform.translation();
    double angle = transform.rotation();
    double x_ph = vec.x;
    double y_ph = vec.y;

    REQUIRE( angle == 90.0 );
    REQUIRE( x_ph == 2.0 );
    REQUIRE( y_ph == 3.0 );
}

TEST_CASE("Test Apply Transform for Vector2D", "[transform]") {
    turtlelib::Vector2D init_vect;
    turtlelib::Vector2D new_vect;
    double angle_init = turtlelib::deg2rad(180.0);
    init_vect.x = 2.0;
    init_vect.y = 4.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(angle_init);
    new_vect = transform(init_vect);

    REQUIRE(new_vect.x== Approx(-2.0));
    REQUIRE(new_vect.y==Approx(-4.0));
}

TEST_CASE("Test Apply Transform for Twist2D", "[transform]") {
    turtlelib::Twist2D init_twist;
    turtlelib::Twist2D new_twist;
    double angle_init = turtlelib::deg2rad(180.0);
    init_twist.theta_dot = 1.0;
    init_twist.x_dot = 2.0;
    init_twist.y_dot = 4.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(angle_init);
    new_twist = transform(init_twist);
    REQUIRE(new_twist.theta_dot== Approx(1.0));
    REQUIRE(new_twist.x_dot== Approx(-2.0));
    REQUIRE(new_twist.y_dot== Approx(-4.0));
}

TEST_CASE("Test Inverse of a Transform", "[transform]") {
    double angle_init = turtlelib::deg2rad(180.0);
    turtlelib::Vector2D init_vect;
    init_vect.x = 2.0;
    init_vect.y = 4.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(init_vect,angle_init);
    turtlelib::Transform2D inv_transform = transform.inv();
    turtlelib::Vector2D inv_vect = inv_transform.translation();
    double inv_angle = inv_transform.rotation();

    REQUIRE(inv_vect.x== Approx(2.0));
    REQUIRE(inv_vect.y== Approx(4.0));
    REQUIRE(inv_angle== Approx(-turtlelib::deg2rad(180.0)));
}

TEST_CASE("Test Multiply Equals", "[transform]") {
    turtlelib::Vector2D vect1;
    turtlelib::Vector2D vect2;
    turtlelib::Vector2D translational_comp;
    double angle1 = turtlelib::deg2rad(90.0);
    double angle2 = turtlelib::deg2rad(90.0);
    double rotational_comp;
    vect1.x = 1;
    vect1.y = 2;
    vect2.x = 3;
    vect2.y = 4;
    turtlelib::Transform2D transform1 = turtlelib::Transform2D(vect1,angle1);
    turtlelib::Transform2D transform2 = turtlelib::Transform2D(vect2,angle2);
    transform1 *= transform2;
    translational_comp = transform1.translation();
    rotational_comp = transform1.rotation();

    REQUIRE(translational_comp.x== Approx(-3.0));
    REQUIRE(translational_comp.y== Approx(5.0));
    REQUIRE(rotational_comp== turtlelib::deg2rad(180.0));

}

TEST_CASE("Test Translation and Rotational Functions", "[transform]") {
    turtlelib::Vector2D vect;
    double angle = turtlelib::deg2rad(90.0);
    vect.x = 6.0;
    vect.y = 12.0;
    turtlelib::Transform2D transform = turtlelib::Transform2D(vect,angle);

    SECTION( "Testing Translational components" ) {
        turtlelib::Vector2D vect_comp;
        vect_comp = transform.translation();
        REQUIRE(vect_comp.x== Approx(6.0));
        REQUIRE(vect_comp.y== Approx(12.0));
    }

    SECTION( "Testing Rotational components" ) {
        double angle_comp;
        angle_comp = transform.rotation();
        REQUIRE(angle_comp== turtlelib::deg2rad(90.0));
    }
}

TEST_CASE("Test Input stream", "[transform]") {
    std::stringstream ss;

    SECTION( "Just numbers" ) {
        ss << 90 << ' '<< 0 << ' ' << 1;
        int deg,x,y;
        ss >> deg >>x>>y;
        REQUIRE(deg == 90);
        REQUIRE(x == 0);
        REQUIRE(y == 1);
    }

    SECTION( "Strings added" ) {
        ss.peek();
        ss << "deg: " << 90 << " x: "<< 0 << " y: " << 1;
        int deg,x,y;
        ss >> deg >>x>>y;
        REQUIRE(deg == 90);
        REQUIRE(x == 0);
        REQUIRE(y == 1);
    }
}

TEST_CASE("Test Output stream", "[transform]") {
    std::stringstream ss;
    std::string s("deg: 90 x: 0 y: 1");
    ss << "deg: " << 90 << " x: "<< 0 << " y: " << 1; 
    REQUIRE(s == ss.str());
}

TEST_CASE("Test Multiply two transforms", "[transform]") {
    turtlelib::Vector2D vect1;
    turtlelib::Vector2D vect2;
    turtlelib::Vector2D translational_comp;
    double angle1 = turtlelib::deg2rad(45.0);
    double angle2 = turtlelib::deg2rad(45.0);
    vect1.x = 4;
    vect1.y = 5;
    vect2.x = 1;
    vect2.y = 2;
    turtlelib::Transform2D transform1 = turtlelib::Transform2D(vect1,angle1);
    turtlelib::Transform2D transform2 = turtlelib::Transform2D(vect2,angle2);
    turtlelib::Transform2D product = transform1*transform2;
    turtlelib::Vector2D prod_vect = product.translation();
    double prod_angle = product.rotation();
    REQUIRE(prod_vect.x == Approx((4-sqrt(2)/2)));
    REQUIRE(prod_vect.y == Approx((5+3*sqrt(2)/2)));
    REQUIRE(prod_angle == (turtlelib::PI/2));
}
