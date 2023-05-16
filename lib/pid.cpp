#include <iostream>
#include <Eigen/Dense>

using namespace std;

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

