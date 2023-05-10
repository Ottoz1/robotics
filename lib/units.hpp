#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

#ifndef CONSTANTS_H
#define CONSTANTS_H

// Motor and gear ratios
const float Motor_C = 4096.0;   // Motor count per revolution
const float Gearbox1 = 6.6/1.0; // Gearbox 1 ratio
const float Motor_C2 = Motor_C * Gearbox1; // Motor count per revolution after gearbox 1
const float Gearbox2 = 24.0/1.0; // Gearbox 2 ratio
const float Wheel_C = Motor_C2 * Gearbox2; // Motor count per revolution after gearbox 2

// Measurement constants
const float WHEEL_BASE = 99.33;  // Distance between wheels (in mm)
const float WHEEL_DIAMETER = 33.17;  // Diameter of the wheels (in mm)

#endif