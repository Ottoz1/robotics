#include "odometry.hpp"
#include "writeToFile.hpp"

using namespace std;
using namespace Eigen;

VectorXf pose_odometry(3);
MatrixXf cov_odometry(3, 3);

void init_odometry(VectorXf start){
    pose_odometry = start;
    cov_odometry = MatrixXf::Zero(3, 3);
    cov_odometry << 1, 0, 0,
         0, 1, 0,
         0, 0, ((1 * M_PI) / 180) * ((1 * M_PI) / 180); // 1 degree
}

void update_odometry_pose(){
    // Change in relative movement
    float dD = get_delta_D();   // Change in distance
    float dT = get_delta_theta();   // Change in angle
    float T = pose_odometry(2);  // Current angle

    // Change in position
    float dx = dD * cos(T + (dT / 2));
    float dy = dD * sin(T + (dT / 2));

    // Update position
    pose_odometry(0) += dx;  // X
    pose_odometry(1) += dy;  // Y
    pose_odometry(2) += dT;  // Theta
    pose_odometry(2) = fmod(pose_odometry(2), 2 * M_PI);  // Wrap theta between 0 and 2pi

    //******************
    // Update covariance
    float a = fmod(pose_odometry(2) + dT/2, 2 * M_PI); // Angle plus half the change in angle

    //Error new = Aj * Error old * Aj' + Bj * C * Bj'
    MatrixXf Aj = MatrixXf::Zero(3, 3);    // State variable jacobian
    Aj << 1.0, 0.0, -dD * sin(a),
        0.0, 1.0, dD * cos(a),
        0.0, 0.0, 1.0;            

    MatrixXf Bj = MatrixXf::Zero(3, 2);; // Measurement jacobian
    Bj << cos(a), (-dD / 2) * sin(a),
        sin(a), (dD / 2) * cos(a),
        0.0,         1.0;

    MatrixXf C = MatrixXf::Zero(2, 2);;
    C << (pow(SIGMAr, 2) + pow(SIGMAl, 2)) / 4, 0.0,
        0.0, (pow(SIGMAr, 2) + pow(SIGMAl, 2)) / pow(WHEEL_BASE, 2);

    MatrixXf old_cov = cov_odometry;

    cov_odometry = Aj * old_cov * Aj.transpose() + Bj * C * Bj.transpose();

    //Save timestamp
    //append_odometry(pose_odometry, cov_odometry);
}

//GET AND SET FUNCTIONS
VectorXf get_odometry_pose(){
    return pose_odometry;
}

MatrixXf get_odometry_cov(){
    return cov_odometry;
}

void set_odometry_pose(VectorXf new_pose){
    pose_odometry = new_pose;
}

void set_odometry_cov(MatrixXf new_cov){
    cov_odometry = new_cov;
}
