#include <iostream>
#include <armadillo>
#include <nuslam/nuslam.hpp>
#include "turtlelib/diff_drive.hpp"
// using namespace std;
// using namespace arma;

// Armadillo documentation is available at:
// http://arma.sourceforge.net/docs.html

// NOTE: the C++11 "auto" keyword is not recommended for use with Armadillo objects and functions

int
main()
  {
    // cout << "Armadillo version: " << arma_version::as_string() << endl;

    // // construct a matrix according to given size and form of element initialisation
    // mat A(2,3,fill::zeros);

    // // .n_rows and .n_cols are read only
    // cout << "A.n_rows: " << A.n_rows << endl;
    // cout << "A.n_cols: " << A.n_cols << endl;

    // A(1,2) = 456.0;  // access an element (indexing starts at 0)
    // A.print("A:");    

    // A = 5.0;         // scalars are treated as a 1x1 matrix
    // A.print("A:");

    // A.set_size(4,5); // change the size (data is not preserved)

    // A.fill(5.0);     // set all elements to a specific value
    // A.print("A:");

    // A = { { 0.165300, 0.454037, 0.995795, 0.124098, 0.047084 },
    //     { 0.688782, 0.036549, 0.552848, 0.937664, 0.866401 },
    //     { 0.348740, 0.479388, 0.506228, 0.145673, 0.491547 },
    //     { 0.148678, 0.682258, 0.571154, 0.874724, 0.444632 },
    //     { 0.245726, 0.595218, 0.409327, 0.367827, 0.385736 } };
        
    // A.print("A:");

    // testing calculate_state_transition
    int n = 5;
    arma::mat A;
    arma::mat I = arma::mat(2*n+3,2*n+3,arma::fill::eye);

    arma::mat tl;
    tl = {  {0.0,0.0,0.0},
            {3.0,0.0,0.0},
            {5.0,0.0,0.0}  };
    arma::mat tr(3,2*n,arma::fill::zeros);
    arma::mat br(2*n,2*n,arma::fill::zeros);
    arma::mat bl(2*n,3,arma::fill::zeros);

    auto joined_top  = std::move(arma::join_rows( tl, tr ));
    auto joined_bot  = std::move(arma::join_rows( bl, br ));
    auto joined_all  = std::move(arma::join_cols( joined_top, joined_bot ));

    A = joined_all + I;
    A.print();
    nuslam::EKF a = nuslam::EKF();
    arma::mat b = a.calculate_updated_state(5.0,0.0);
    b.print();
  return 0;
  }