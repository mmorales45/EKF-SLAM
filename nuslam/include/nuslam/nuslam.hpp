#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
#include <iostream>
#include <armadillo>
#include <iosfwd> 


namespace nuslam
{
    class EKF
    {
    public:

        EKF();

        arma::mat calculate_updated_state(double delta_x, double delta_t);

        // arma::mat calculate_transition(double delta_x, double delta_t);
    private:
        /*
        variables needed?
        xk,zk,x_hat_k,z_hat_k,


        */
        arma::mat current_state;
        arma::mat qt;
        // arma::mat ut; //this is the twist
        arma::mat mt;

    };
}

#endif