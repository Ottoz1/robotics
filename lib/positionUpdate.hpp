#include "ravLidar.hpp"
#include "cox.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

extern VectorXf pos;
extern MatrixXf cov;
extern int new_pos_ready;

int positionUpdater();