#include "positionUpdate.hpp"
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
        if(dataReady){
            posO = get_odometry_pose();
            covO = get_odometry_cov();

            posC = posO;
            cart = polar_to_cart(points);
            cart = transform_points(cart, posO);    // Laser to world frame

            transformation = cox_linefit(cart, line_segments, 100, &covC);

            posC(0) += transformation(0);
            posC(1) += transformation(1);
            posC(2) += transformation(2);

            //Print this please
            printf("Cx=%f Cy=%f Ctheta=%f\n",posC(0),posC(1),posC(2));
            printf("CovarianceC: \n");
            cout << covC << endl;

            dataReady=0;
        }
    }
}
