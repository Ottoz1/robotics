#pragma once

double calculatePID(double error, double integral, double prev_error, double Kp, double Ki, double Kd);

double calculatePI(double error, double integral, double prev_error, double Kp, double Ki);

double calculatePD(double error, double prev_error, double Kp, double Kd) ;

double calculateP(double error, double Kp);