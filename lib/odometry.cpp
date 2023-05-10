#include "odometry.hpp"

using namespace std;
using namespace Eigen;

void init_odometry(){
    pose = START_POSE;
}

VectorXf get_odometry_pose(){
    return pose;
}