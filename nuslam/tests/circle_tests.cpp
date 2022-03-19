#include <catch_ros/catch.hpp>
#include <turtlelib/rigid2d.hpp>
#include <nuslam/CircleFitting.hpp>
#include <string>       
#include <iostream>    
#include <sstream> 

// using namespace nuslam;

TEST_CASE("Test 1", "[nuslam]") {
    nuslam::nuCircles makeCircles;
    std::vector<std::vector<turtlelib::Vector2D>> main_cluster;
    std::vector<turtlelib::Vector2D> cluster;
    turtlelib::Vector2D point1, point2, point3, point4, point5, point6;
    std::vector<double> R_array;
    std::vector<turtlelib::Vector2D> xy_COORDS;

    point1 = {1,7};
    point2 = {2,6};
    point3 = {5,8};
    point4 = {7,7};
    point5 = {9,5};
    point6 = {3,7};
    
    cluster.push_back(point1);
    cluster.push_back(point2);
    cluster.push_back(point3);
    cluster.push_back(point4);
    cluster.push_back(point5);
    cluster.push_back(point6);

    main_cluster.push_back(cluster);
    makeCircles = nuslam::nuCircles(main_cluster);
    makeCircles.CircleFitting();

    R_array = makeCircles.getR_array();
    xy_COORDS = makeCircles.getCirclePosition();

    double R = R_array.at(0);
    double x,y;
    x = xy_COORDS.at(0).x;
    y = xy_COORDS.at(0).y;


    CHECK( turtlelib::almost_equal(R, 4.8275, 1.0e-4)); 
    CHECK( turtlelib::almost_equal(x, 4.615482, 1.0e-4)); 
    CHECK( turtlelib::almost_equal(y, 2.807354, 1.0e-4)); 
}

TEST_CASE("Test 2", "[nuslam]") {
    nuslam::nuCircles makeCircles;
    std::vector<std::vector<turtlelib::Vector2D>> main_cluster;
    std::vector<turtlelib::Vector2D> cluster;
    turtlelib::Vector2D point1, point2, point3, point4;
    std::vector<double> R_array;
    std::vector<turtlelib::Vector2D> xy_COORDS;

    point1 = {-1,0};
    point2 = {-0.3,-0.06};
    point3 = {0.3,0.1};
    point4 = {1,0};
    
    cluster.push_back(point1);
    cluster.push_back(point2);
    cluster.push_back(point3);
    cluster.push_back(point4);

    main_cluster.push_back(cluster);
    makeCircles = nuslam::nuCircles(main_cluster);
    makeCircles.CircleFitting();

    R_array = makeCircles.getR_array();
    xy_COORDS = makeCircles.getCirclePosition();

    double R = R_array.at(0);
    double x,y;
    x = xy_COORDS.at(0).x;
    y = xy_COORDS.at(0).y;


    CHECK( turtlelib::almost_equal(R, 22.17979, 1.0e-4)); 
    CHECK( turtlelib::almost_equal(x, 0.4908357, 1.0e-4)); 
    CHECK( turtlelib::almost_equal(y, -22.15212, 1.0e-4)); 
}