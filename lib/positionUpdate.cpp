#include "positionUpdate.hpp"
using namespace std;
using namespace Eigen;

int positionUpdater(){
    VectorXf posC = VectorXf::Zero(3);
    MatrixXf covC = MatrixXf::Zero(3,3);
    VectorXf posO = VectorXf::Zero(3);
    MatrixXf covO = MatrixXf::Zero(3,3);
    VectorXf posK = VectorXf::Zero(3);
    MatrixXf covK = MatrixXf::Zero(3,3);
    MatrixXf cart;
    VectorXf transformation;
    while(1){

        posO = get_odometry_pose();
        covO = get_odometry_cov();

        printf("Ox=%f Oy=%f Otheta=%f\n",posO(0),posO(1),posO(2));
        printf("Odometry covariance: \n");
        cout << covO << endl;
        printf("__________________________\n");

        if(dataReady){
            posC = posO;
            cart = polar_to_cart(points);
            cart = transform_points(cart, posO);    // Laser to world frame

            transformation = cox_linefit(cart, line_segments, 100, &covC);

            posC(0) += transformation(0);
            posC(1) += transformation(1);
            posC(2) += transformation(2);

            //Print this please
            printf("Cx=%f Cy=%f Ctheta=%f\n",poseC(0),poseC(1),poseC(2));
            printf("CovarianceC: \n");
            cout << covC << endl;

            dataReady=0;
        }
    }
}
