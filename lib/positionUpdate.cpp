#include "positionUpdate.hpp"
#include "writeToFile.hpp"
#include "motors.hpp"
using namespace std;
using namespace Eigen;

int positionUpdater(){
    points;
    dataReady;
    VectorXf posC = VectorXf::Zero(3);
    MatrixXf covC = MatrixXf::Zero(3,3);
    VectorXf posO = VectorXf::Zero(3);
    MatrixXf covO = MatrixXf::Zero(3,3);
    VectorXf posK = VectorXf::Zero(3);
    MatrixXf covK = MatrixXf::Zero(3,3);
    MatrixXf cart;
    VectorXf transformation;
    MatrixXf line_segments = generate_lines();
    while(1){
        if(dataReady && !turning){
            posO = get_odometry_pose();
            covO = get_odometry_cov();

            posC = posO;
            cart = polar_to_cart(points);
            cart = transform_points(cart, posO);    // Laser to world frame

            transformation = cox_linefit(cart, line_segments, 20, &covC);

            // Check if transformation is nan
            if(transformation(0) != transformation(0) || transformation(1) != transformation(1) || transformation(2) != transformation(2)){
                dataReady=0;
                continue;
            }

            // Check if transformation is -1000000, -1000000, -1000000
            if(transformation(0) == -1000000 && transformation(1) == -1000000 && transformation(2) == -1000000){
                dataReady=0;
                continue;
            }

            cart = transform_points(cart, transformation);    // World to camera frame
            plot(cart);

            posC(0) += transformation(0);
            posC(1) += transformation(1);
            posC(2) += transformation(2);

            posK = kalman_combine_pose(posC, posO, covC, covO);
            covK = kalman_combine_cov(covC, covO);
            
            cout << "posK: " << posK << endl;

            cout << "__________________________" << endl;

            set_odometry_pose(posK);
            set_odometry_cov(covK);

            append_cox(posC, covC);
            append_kalman(posK, covK);

            dataReady=0;
        }
    }
}
