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
        /// \brief Create an object of type nuCircles with nothing initialized.
        nuCircles();

        /// \brief Create an object for the the class with a vector of vector of Vector2D.
        nuCircles(std::vector<std::vector<turtlelib::Vector2D>> main_cluster);
        
        /// \brief Create circles by calculating radius and coordinates using x and y values.
        void CircleFitting();

        /// \brief Classify if the cluster are circles or not.
        /// \return new_Clusters- the updated main cluster without the false circle clusters.
        std::vector<std::vector<turtlelib::Vector2D>> ClassifyCircles();

        /// \brief Function to retrieve private variable R_array.
        /// \return R_array- Vector of radii of the clusters that make a true circle.
        std::vector<double> getR_array();

        /// \brief Function to retrieve private variable xy_COORDS.
        /// \return xy_COORDS- Vector of circle made by the cluster.
        std::vector<turtlelib::Vector2D> getCirclePosition();

    private:
        std::vector<std::vector<turtlelib::Vector2D>> clusterStore;
        std::vector<double> R_array;
        std::vector<turtlelib::Vector2D> xy_COORDS;
    };
}
#endif 