#include "kalman.hpp"

using namespace std;
using namespace Eigen;

VectorXf kalman_combine_pose(VectorXf pos1, VectorXf pos2, MatrixXf cov1, MatrixXf cov2){
    MatrixXf cov_sum = cov1 + cov2;
    MatrixXf factor1 = cov2 * cov_sum.inverse();
    MatrixXf factor2 = cov1 * cov_sum.inverse();
    VectorXf pos = factor1 * pos1 + factor2 * pos2;
    return pos;
}

MatrixXf kalman_combine_cov(MatrixXf cov1, MatrixXf cov2){
    MatrixXf cov1_inv = cov1.inverse();
    MatrixXf cov2_inv = cov2.inverse();
    MatrixXf cov = (cov1_inv + cov2_inv).inverse();
    return cov;
}

void run_kalman(){
    VectorXf pose1(3);
    VectorXf pose2(3);
    MatrixXf cov1(3,3);
    MatrixXf cov2(3,3);

    pose1 << 61.521072, 1286.232300, 0.423230;
    pose2 << 254.676437, 1240.765015, 0.648712;
    cov1 << 0.0111495, 0.000811569, -0.00000178384,
            0.000811567, 0.010602, -0.00000466781,
            -0.00000178384, 0.00000466781, 0.0000000102599;
    cov2 << 1.20202, -0.480881, -0.00769998,
            -0.480881, 2.21953, 0.0193615,
            -0.00769998, 0.0193615, 0.000309192;

    VectorXf pose = kalman_combine_pose(pose1, pose2, cov1, cov2);
    MatrixXf cov = kalman_combine_cov(cov1, cov2);

    cout << "pose: " << endl << pose << endl;
    cout << "cov: " << endl << cov << endl;
}