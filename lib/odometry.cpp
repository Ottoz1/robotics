#include "odometry.hpp"

using namespace std;
using namespace Eigen;

VectorXf pose_odometry(3);

void init_odometry(VectorXf start){
    pose_odometry = start;
}

VectorXf get_odometry_pose(){
    return pose_odometry;
}

void set_odometry_pose(VectorXf new_pose){
    pose_odometry = new_pose;
}

void update_odometry_pose(){
    float dD = get_delta_D();   // Change in distance
    float dT = get_delta_theta();   // Change in angle
    float T = pose_odometry(2);  // Current angle

    // Convert to X and Y
    float dx = dD * cos(T);
    float dy = dD * sin(T);

    // Update the pose
    pose_odometry(0) += dx;  // X
    pose_odometry(1) += dy;  // Y
    //pose(2) += dT;  // Theta
}