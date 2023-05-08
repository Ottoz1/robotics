#include "lib/matplotlibcpp.h"
#include "speedProfile.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;

int main() {

    Eigen::Vector2f startPos(0, 0);
    Eigen::Vector2f endPos(-30, 14);

    speedProfile sp(startPos, endPos, 0);

    sp.run();

    std::vector<float> timeStamps;
    std::vector<float> speeds = sp.getSpeeds();
    std::vector<float> positions = sp.getPositions();
    std::vector<std::vector<float>> wheelVelocities = sp.getWheelVelocities();
    std::vector<float> rightWheelVelocities;
    std::vector<float> leftWheelVelocities;

    cout << "speeds: " << endl;
    for (int i = 0; i < wheelVelocities.size(); i++){
        cout << wheelVelocities[i][0] << " " << wheelVelocities[i][1] << endl;
        rightWheelVelocities.push_back(wheelVelocities[i][1]);
        leftWheelVelocities.push_back(wheelVelocities[i][0]);
    }

    for (int i = 0; i < rightWheelVelocities.size(); i++){
        timeStamps.push_back(i * 0.01);
    }

    //print angle between startPos and endPos
    cout << "angle: " << sp.getAngle()<< endl;

    matplotlibcpp::plot(timeStamps, rightWheelVelocities);
    matplotlibcpp::plot(timeStamps, leftWheelVelocities);
    matplotlibcpp::title("Speed and Position over time");
    matplotlibcpp::xlabel("Time (s)");
    matplotlibcpp::ylabel("right wheeel (m/s)(blue) left wheel (m)(orange)");

    matplotlibcpp::show();
}