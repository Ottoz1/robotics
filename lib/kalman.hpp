#include <thread>
#include <Eigen/Dense>
#include <unistd.h>
#include <stdio.h>
#include "units.hpp"

VectorXf kalman_combine_pos(VectorXf pos1, VectorXf pos2, MatrixXf cov1, MatrixXf cov2);
MatrixXf kalman_combine_cov(MatrixXf cov1, MatrixXf cov2);
void run_kalman();
