#include <iostream>
#include <Eigen/Dense>

using namespace std;

const double Kp_distance = 1.0;    // Proportional gain for distance control
const double Ki_distance = 0.1;    // Integral gain for distance control
const double Kd_distance = 0.01;   // Derivative gain for distance control
const double Kp_theta = 1.0;       // Proportional gain for angle control
const double Ki_theta = 0.1;       // Integral gain for angle control
const double Kd_theta = 0.01;      // Derivative gain for angle control

double calculatePID(double error, double integral, double prev_error, double Kp, double Ki, double Kd) {
    double p = Kp * error;
    double i = Ki * integral;
    double d = Kd * (error - prev_error);
    return p + i + d;
}

double calculatePI(double error, double integral, double Kp, double Ki) {
    double p = Kp * error;
    double i = Ki * integral;
    return p + i;
}

double calculatePD(double error, double prev_error, double Kp, double Kd) {
    double p = Kp * error;
    double d = Kd * (error - prev_error);
    return p + d;
}

double calculateP(double error, double Kp) {
    return Kp * error;
}

