#include <iostream>
#include <armadillo>
#include<cmath>
#include <nuslam/nuslam.hpp>
// #include "turtlelib/diff_drive.hpp"

namespace nuslam
{
    // odometry model
    EKF::EKF()
    {
        arma::mat current_state(3, 1, arma::fill::zeros);
        current_state(0,0) = 0.0;
        current_state(1,0) = 0.0;
        current_state(2,0) = 0.0;
    }

    arma::mat EKF::calculate_updated_state(double delta_x, double delta_t)
    {
        int n = 0;
        double x_cos;
        double x_sin;  
        arma::mat A;
        arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);

        if(delta_t == 0.0)
        {
            x_cos = -delta_x*sin(delta_t);
            x_sin = delta_x*cos(delta_t);
        }
        else
        {
            x_cos = -(delta_x/delta_t)*cos(delta_x) + (delta_x/delta_t)*cos(delta_t);
            x_sin = -(delta_x/delta_t)*sin(delta_x) + (delta_x/delta_t)*sin(delta_t);
        }
        arma::mat tl;
        tl = {  {0.0,0.0,0.0},
                {x_cos,0.0,0.0},
                {x_sin,0.0,0.0}  };
        arma::mat tr(3,2*n,arma::fill::zeros);
        arma::mat br(2*n,2*n,arma::fill::zeros);
        arma::mat bl(2*n,3,arma::fill::zeros);
        
        auto joined_top  = std::move(arma::join_rows( tl, tr ));
        auto joined_bot  = std::move(arma::join_rows( bl, br ));
        auto joined_all  = std::move(arma::join_cols( joined_top, joined_bot ));

        A = I + joined_all;
        return A;
    }

    // arma::mat calculate_transition(double delta_x, double delta_t)
    // {
    //     arma::mat T_wp_prime =  arma::mat(3,1,arma::fill::zeros);
    //     arma::mat w_t =  arma::mat(3,1,arma::fill::zeros);

    //     if ()


    // }
}