#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <cassert>
#include "units.hpp"

using namespace std;

class speedProfile{
    public:
        float maxVelocity;
        float dt;
        float acceleration;
        Eigen::Vector2f startPos;
        Eigen::Vector2f endPos;
        float direction; // current direction of the robot

        speedProfile(float maxVelocity, float dt, float acceleration, Eigen::Vector2f startPos, Eigen::Vector2f endPos, float direction);

        ~speedProfile();

        void accelerate();
        void holdVelocity();
        void decelerate();

        float getTheta(); // returns the angle theta between the current direction and the direction to the end position
        float getDistance(); // returns the distance between startPos and endPos
        vector<float> getSpeedProfile(float dist);

        //will return velocities of both wheels that will move the robot by the size of the vector between startPos and endPos
        vector<vector<float>> walk();
        //will return velocities of both wheels that will turn the robot by theta
        vector<vector<float>> turn(); 
        void run();

        vector<vector<float>> getWheelVelocities();
        vector<float> getPositions();
        vector<float> getVelocities();
        vector<vector<float>> forwardKinematics(vector<float> rightWheelVel, vector<float> leftWheelVel, float wheelRadius, float x, float y, float theta);

    private:
        float wheelBase = WHEEL_BASE;
        std::vector<float> velocities;
        float velocity;
        // wheelVelocities = {leftWheelVelocities, rightWheelVelocities}
        std::vector<vector<float>> wheelVelocities;
        std::vector<float> positions;
        int status; // 0 = not initialized, 1 = running, 2 = finished

};