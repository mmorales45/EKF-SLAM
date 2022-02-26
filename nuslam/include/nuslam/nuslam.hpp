#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
#include <iostream>
#include <armadillo>
#include <iosfwd> 
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/rigid2d.hpp>


namespace nuslam
{
    struct bearing_measure
    {
        double r_j = 0.0;

        double phi_j = 0.0;
    };

    class KalmanFilter
    {
    public:

        KalmanFilter();

        arma::mat calculate_updated_state(turtlelib::Twist2D twist);

        arma::mat calculate_transition(turtlelib::Twist2D twist);

        arma::mat calculate_h(int j);

        arma::mat calculate_H(int j);

        arma::mat calculate_Q_est();

        arma::mat predict(turtlelib::Twist2D twist);

        arma::mat update(int j, arma::mat z);

        arma::mat Landmark_Initialization(int robot_id, turtlelib::Vector2D coords);

    private:
        /*
        variables needed?
        xk,zk,x_hat_k,z_hat_k,


        */
        int n;
        arma::mat current_state;

        arma::mat qt;
        // arma::mat ut; //this is the twist
        arma::mat mt;

        arma::mat predict_state_est;    
        arma::mat predict_cov_est;

        bearing_measure current_measure;

        arma::mat Q;
        arma::mat R;

        arma::mat sigma;

    };
}

#endif