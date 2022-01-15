#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rigid2d.hpp"

TEST_CASE("Identity Transform", "[transform]") {
    turtlelib::Transform2D transform = turtlelib::Transform2D();
    turtlelib::Vector2D vec = transform.translation();
    double angle = transform.rotation();
    double x_ph = vec.x;
    double y_ph = vec.x;

    REQUIRE( angle == 0 );
    REQUIRE( x_ph == 0 );
    REQUIRE( y_ph == 0 );
}

TEST_CASE("Translational Only Transform", "[transform]") {
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

TEST_CASE("Rotational Only Transform", "[transform]") {
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

TEST_CASE("Both Translational and Rotational Transform", "[transform]") {
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
