#include "lib/matplotlibcpp.h"
#include "speedProfile.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

int main() {

    Eigen::Vector2f startPos(-50, 50);
    Eigen::Vector2f endPos(-200, 0);
    float direction = 0;
    vector<float> timeStamps;

    speedProfile sp(10, 0.01 , 2, startPos, endPos, direction);
    sp.run();
    vector<vector<float>> wheelVelocities = sp.getWheelVelocities();
    vector<float> rightWheelVelocities;
    vector<float> leftWheelVelocities;
    for (int i = 0; i < wheelVelocities.size(); i++){
        //cout << "left wheel velocity: " << wheelVelocities[i][0] << " right wheel velocity: " << wheelVelocities[i][1] << endl;
        rightWheelVelocities.push_back(wheelVelocities[i][1]);
        leftWheelVelocities.push_back(wheelVelocities[i][0]);
        //cout << "right wheel velocity: " << rightWheelVelocities[i] << " left wheel velocity: " << leftWheelVelocities[i] << endl;
    }
    vector<vector<float>> positions;
    positions = sp.forwardKinematics(rightWheelVelocities, leftWheelVelocities, 1, startPos[0], startPos[1], direction);
    //vector<float> positions = sp.getPositions();
    vector<float> velocities = sp.getVelocities();

    vector<float> x_val;
    vector<float> y_val;
    vector<float> theta_val;

    for (int i = 0; i < rightWheelVelocities.size(); i++){
        timeStamps.push_back(i * 0.01);
        x_val.push_back(positions[i][0]);
        y_val.push_back(positions[i][1]);
        theta_val.push_back(positions[i][2]);
    }

    //print final position
    cout << "Final Position: " << positions[positions.size() - 1][0] << ", " << positions[positions.size() - 1][1] << endl;

    //print angle between startPos and endPos
    cout << "Angle: " << sp.getTheta() << endl;

    matplotlibcpp::plot(timeStamps, rightWheelVelocities);
    matplotlibcpp::plot(timeStamps, leftWheelVelocities);
    //scatter plot x_val and y_val as arrows with the direction theta_val
    //matplotlibcpp::scatter(x_val, y_val);
    matplotlibcpp::title("Speed and Position over time");
    matplotlibcpp::xlabel("Time (s)");
    matplotlibcpp::ylabel("right wheeel (m/s)(blue) left wheel (m)(orange)");

    matplotlibcpp::show();
}