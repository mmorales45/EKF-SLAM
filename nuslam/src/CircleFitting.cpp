#include<iosfwd>
#include <cmath>
#include <armadillo>
#include <iostream>
#include <nuslam/CircleFitting.hpp>



namespace nuslam
{
    nuCircles::nuCircles()
    {
    }

    nuCircles::nuCircles(std::vector<std::vector<turtlelib::Vector2D>> main_cluster)
    {
        clusterStore = main_cluster;
    }

    void nuCircles::CircleFitting()
    {
        int size_main = clusterStore.size();
        R_array.clear();
        xy_COORDS.clear();
        //Resize radius and coordinate vectors.
        R_array.resize(size_main);
        xy_COORDS.resize(size_main);
        for (int i = 0;i<size_main;i++)
        {
            //Create a cluster to loop at from the main set of clusters.
            std::vector<turtlelib::Vector2D> currentCluster = clusterStore[i];
            int size_cluster = currentCluster.size();

            double x_sums = 0.0;
            double y_sums = 0.0;
            //loop through every cluster to calculate the sums of both x and y values.
            for (int j = 0;j< size_cluster;j++)
            {
                turtlelib::Vector2D currentXY = currentCluster[j];
                x_sums = x_sums + currentXY.x;
                y_sums = y_sums + currentXY.y;
            }
            //calculate means of both x and y.
            double x_mean,y_mean;
            x_mean = x_sums/size_cluster;
            y_mean = y_sums/size_cluster;

            //Create empty vectors to append the centroid values.
            std::vector<double> x_cent;
            std::vector<double> y_cent;
            std::vector<double> z_cent;
            x_cent.resize(size_cluster);
            y_cent.resize(size_cluster);
            z_cent.resize(size_cluster);

            //Loop through every value to calculate zi and centroid values.
            double z_sums = 0;
            for (int j = 0;j< size_cluster;j++)
            {
                turtlelib::Vector2D currentXY = currentCluster[j];
                double x_diff, y_diff, z_i;
                x_diff = currentXY.x - x_mean;
                y_diff = currentXY.y - y_mean;
                z_i = pow(x_diff,2) + pow(y_diff,2);

                x_cent.at(j) = x_diff;
                y_cent.at(j) = y_diff;
                z_cent.at(j) = z_i;
                z_sums = z_sums + z_i;
            }

            //calculate z mean.
            double z_mean = 0;
            z_mean = z_sums/size_cluster;
            //create Z matrix and loop through every row to fill up each cell.
            arma::mat Z(size_cluster,4,arma::fill::ones);
            for (int k = 0;k< size_cluster;k++)
            {
                Z(k,0) = z_cent.at(k);
                Z(k,1) = x_cent.at(k);
                Z(k,2) = y_cent.at(k);
                Z(k,3) = 1;
            }

            //create M matrix.
            arma::mat M(4,4,arma::fill::zeros);
            M = (1/size_cluster) * Z.t() * Z;
            //create H matrix
            arma::mat H(4,4,arma::fill::zeros);

            H(0,0) = 8*z_mean;
            H(0,3) = 2;
            H(1,1) = 1;
            H(2,2) = 1;
            H(3,0) = 2;
            // create Hinv matrix
            arma::mat H_inv(4,4,arma::fill::zeros);

            H_inv(0,3) = 0.5;
            H_inv(1,1) = 1;
            H_inv(2,2) = 1;
            H_inv(3,0) = 0.5;
            H_inv(3,3) = -2*z_mean;

            //calculate svd using arma
            arma::mat U;
            arma::vec sigma; //cant be a mat, needs to be vec
            arma::mat V;
            arma::svd(U,sigma,V,Z);
            // The smallest sigma is the 4th column
            double smallest_sigma = sigma(3);
            //if sigma is less than 0.0001
            arma::vec A;
            if (smallest_sigma < 1.0e-11)
            {
                A = V.col(3); //make A equal to 4th column of V
            }
            else
            {
                // Create Y and calculate it, then find Q
                arma::mat Y;
                Y = V * diagmat(sigma) * V.t();
                arma::mat Q;
                Q = Y * H_inv * Y;
                //Create mat and vec to use when solving for eigenvalues
                arma::vec eigval;
                arma::mat eigvec;

                eig_sym(eigval,eigvec,Q);


                arma::vec A_star_vec;
                double eig_compare = 10.0;
                int iter = 0;
                for (int k = 0;k<eigval.size();k++)
                {
                    if ((eigval[k]>0.0 && eigval[k]<eig_compare))
                    {
                        iter = k;
                        eig_compare = eigval[k];
                    }
                }
                //Get the A star values from eigvec 
                A_star_vec = eigvec.col(iter);
                A = arma::solve(Y,A_star_vec);
            }
            double a,b,R;
            //Calculate a,b to get Radius of circle and positions
            a = -A(1)/(2*A(0));
            b = -A(2)/(2*A(0));
            R = sqrt( (pow(A(1),2) + pow(A(2),2) - (4*A(0)*A(3)))/(4*pow(A(0),2)) );
            //Incorporate the means into position.
            turtlelib::Vector2D circleCenter = {a+x_mean,b+y_mean};
            R_array.at(i) = R;
            xy_COORDS.at(i) = circleCenter;            
        }
    }

    std::vector<std::vector<turtlelib::Vector2D>> nuCircles::ClassifyCircles()
    {
        //Create the mind and max angles for arcs to be circles.
        double min_angle = turtlelib::PI/2;
        double max_angle = 2.35619;
        std::vector<std::vector<turtlelib::Vector2D>> new_Clusters;
        int size_main = clusterStore.size();
        for (int i = 0;i<size_main;i++)
        {
            //Create a cluster variable from the main cluster vector.
            std::vector<turtlelib::Vector2D> currentCluster = clusterStore[i];
            int size_cluster = currentCluster.size();
            turtlelib::Vector2D Point_1,Point_2;
            //Find coordinates of initial and final points in the cluster.
            Point_1 = currentCluster[0];
            Point_2 = currentCluster[size_cluster-1];

            double angle_sums = 0.0;
            std::vector<double> angle_array;
            //Loop through the angles and add them to a vector and add the angles.
            for (int j = 1;j< (size_cluster-1);j++)
            {
                turtlelib::Vector2D new_Point1, new_Point2;
                new_Point1 = {(Point_1.x - currentCluster[j].x),
                                (Point_1.y - currentCluster[j].y)};
                new_Point2 = {(Point_2.x - currentCluster[j].x),
                                (Point_2.y - currentCluster[j].y)};
                double product = turtlelib::dot(new_Point1, new_Point2);
                double mag1 = turtlelib::magnitude(new_Point1);
                double mag2 = turtlelib::magnitude(new_Point2);

                double arc_angle = acos(product/(mag1*mag2));

                angle_sums = angle_sums + arc_angle;
                angle_array.push_back(arc_angle);
            }
            //divide sum of angles by num of points to find mean.
            double angle_mean = 0.0;
            angle_mean = angle_sums / (size_cluster-2);        

            double standard_deviation = 0.0;
            //add standard deviations together.
            for (int k = 0;k<(size_cluster-2);k++)
            {
                double sd_val = pow( (angle_array.at(k)-angle_mean) ,2 );
                standard_deviation = standard_deviation + sd_val;
            }
            double sd_compare = 0.0;
            //find the mean for SD.
            sd_compare = std::sqrt(standard_deviation/((size_cluster-3)));
            
            //If angle of circle fall within the range, append them to the main cluster list.
            if ((angle_mean <= max_angle) && (angle_mean >= min_angle) && (sd_compare<0.25)) 
            {
                new_Clusters.push_back(currentCluster);
            }   
        }
        // true_clusters =  new_Clusters;
        return new_Clusters;
    }
        

    std::vector<double> nuCircles::getR_array()
    {
        return R_array;
    }

    std::vector<turtlelib::Vector2D> nuCircles::getCirclePosition()
    {
        return xy_COORDS;
    }
}