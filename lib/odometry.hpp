#include <thread>
#include <Eigen/Dense>
#include <unistd.h>
#include <stdio.h>
#include "units.hpp"
#include "motors.hpp"

void init_odometry(VectorXf start);
VectorXf get_odometry_pose();
void set_odometry_pose(VectorXf new_pose);
void update_odometry_pose();
