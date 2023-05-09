#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <cassert>

class speedProfile{
    //program variables
    float velocity;
    float acceleration;
    float distance;
    float angle;

    //user specified variables
    Eigen::Vector2f startPos; // start position
    Eigen::Vector2f endPos; // goal position
    float direction; // current direction of robot

    //robot constants
    float sampleTime;
    float maxVelocity;
    float radius;
    float wheelBase;
    float motorValue;

    //mainly for plotting
    std::vector<float> speeds;
    std::vector<float> positions;
    // wheel velocities = {left, right}
    std::vector<std::vector<float>> wheelVelocities;

    //outputs
    //eeeeh temp ugly global kill me
    std::vector<float> profiles;

    std::vector<std::vector<float>> wheelProfiles;

    /*
        0 = not started
        1 = running
        2 = finished
    */
    int status = 0;

    public:
        int setNewGoal(Eigen::Vector2f goal);

        /*
            goal: goal position
            sampleTime: time between each sample
            maxVelocity: maximum velocity
        */
        speedProfile(Eigen::Vector2f startPos, Eigen::Vector2f endPos, float direction);

        ~speedProfile();

        void accelerate();
        void holdVelocity();
        void decelerate();

        void run();

        int getStatus();
        int getAngle();
        int getSampleTime();
        std::vector<float> getSpeeds();
        std::vector<float> getPositions();
        std::vector<std::vector<float>> getWheelVelocities();

        void getSpeedProfile(float dist);

        std::vector<std::vector<float>> calcWheelVelocities();
        std::vector<std::vector<float>> calcWheelTurnVelocities();
        std::vector<std::vector<float>> calcWheelProfiles();
        std::vector<std::vector<float>> calcWheelTurnProfiles();
};