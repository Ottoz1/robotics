#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

class speedProfile{
    //program variables
    float position;
    float velocity;
    float acceleration;
    double breakDistance;
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
    float direction;

    /*
        0 = not started
        1 = running
        2 = finished
    */
    int status = 0;

    public:
        int setNewGoal(float goal);

        /*
            goal: goal position
            sampleTime: time between each sample
            maxVelocity: maximum velocity
        */
        speedProfile(float goal, float sampleTime, float maxVelocity);

        ~speedProfile();

        void accelerate();
        void holdVelocity();
        void decelerate();

        void turn();
        void run();

        int getStatus();

        int getSampleTime();

        std::vector<float> getSpeeds();
        std::vector<float> getPositions();

};