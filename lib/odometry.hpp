#include <thread>
#include <Eigen/Dense>
#include <unistd.h>
#include <stdio.h>
#include "units.hpp"
#include "motors.hpp"

void init_odometry(VectorXf start);
void update_odometry_pose();

//GET AND SET FUNCTIONS
VectorXf get_odometry_pose();
void set_odometry_pose(VectorXf new_pose);
MatrixXf get_odometry_cov();
void set_odometry_cov(MatrixXf new_cov);
