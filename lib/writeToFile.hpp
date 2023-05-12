#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <unistd.h>
#include <stdio.h>

using namespace std;
using namespace Eigen;

void append_odometry(VectorXf pos, MatrixXf cov);
void append_cox(VectorXf pos, MatrixXf cov);
void append_kalman(VectorXf pos, MatrixXf cov);