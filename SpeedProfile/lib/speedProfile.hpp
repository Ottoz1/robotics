#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

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

    //mainly for plotting
    std::vector<float> speeds;
    std::vector<float> positions;

    //outputs 
    // wheel velocities = {left, right}
    std::vector<std::vector<float>> wheelVelocities;

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

        void turn();
        void run();

        int getStatus();
        int getAngle();
        int getSampleTime();

        std::vector<float> getSpeeds();
        std::vector<float> getPositions();

        void getSpeedProfile(float dist);

        std::vector<std::vector<float>> getWheelVelocities();
        std::vector<std::vector<float>> getWheelTurnVelocities();
};