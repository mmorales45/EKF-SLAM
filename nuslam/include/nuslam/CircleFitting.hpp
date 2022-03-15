#ifndef NUCIRCLES_INCLUDE_GUARD_HPP
#define NUCIRCLES_INCLUDE_GUARD_HPP
#include<iosfwd>
#include <cmath>
#include <iostream>
#include <turtlelib/rigid2d.hpp>
#include <vector>
#include <armadillo>

namespace nuslam
{
    class nuCircles
    {
    public:
        nuCircles();
        nuCircles(std::vector<std::vector<turtlelib::Vector2D>> main_cluster);

        void CircleFitting();
        std::vector<std::vector<turtlelib::Vector2D>> ClassifyCircles();

        std::vector<double> getR_array();
        std::vector<turtlelib::Vector2D> getCirclePosition();

    private:
        std::vector<std::vector<turtlelib::Vector2D>> clusterStore;
        std::vector<double> R_array;
        std::vector<turtlelib::Vector2D> xy_COORDS;
    };
}
#endif 