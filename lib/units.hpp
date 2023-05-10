#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

VectorXf START_POSE(3);

const float WHEEL_BASE = 99.33;  // Distance between wheels (in mm)
const float WHEEL_DIAMETER = 33.17;  // Diameter of the wheels (in mm)