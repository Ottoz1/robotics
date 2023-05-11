#include "speedProfile.hpp"

/**
* @brief Construct a new speed Profile::speed Profile object
* @param maxVelocity maximum velocity of the robot
* @param dt time between each sample
* @param acceleration acceleration of the robot
* @param dt time between each sample
* @param startPos starting position of the robot
* @param endPos ending position of the robot
* @param direction current direction of the robot
*
* @return speedProfile object
*/
speedProfile::speedProfile(float maxVelocity, float dt, float acceleration, Eigen::Vector2f startPos, Eigen::Vector2f endPos, float direction){
    this->maxVelocity = maxVelocity;
    this->dt = dt;
    this->acceleration = acceleration;
    this->startPos = startPos;
    this->endPos = endPos;
    this->direction = direction;
    this->status = 0;
}

speedProfile::~speedProfile(){
    //nothing to do here
}

void speedProfile::accelerate(){
    acceleration = abs(acceleration);
    velocity += acceleration * dt;
    return;
}
void speedProfile::holdVelocity(){
    velocity = maxVelocity;
    return;
}
void speedProfile::decelerate(){
    acceleration = -abs(acceleration);
    velocity += acceleration * dt;
    return;
}

//returns the angle theta between the current direction and the direction to the end positions
float speedProfile::getTheta(){
    float theta = atan2(endPos[1] - startPos[1], endPos[0] - startPos[0]) - direction;
    return theta;
}

//returns the distance between startPos and endPos
float speedProfile::getDistance(){
    Eigen::Vector2f directionVector = endPos - startPos;
    return directionVector.norm();
}

vector<float> speedProfile::getSpeedProfile(float dist){
    dist = abs(dist);
    velocity = 0;
    float position = 0;
    double breakDistance = 0;
    status = 1;
    while ((position + breakDistance) <= dist){
       if(velocity < maxVelocity){
            accelerate();
        } 
        else {
            holdVelocity();
        }
        position += velocity * dt;
        breakDistance = pow(velocity, 2) / (2 * abs(acceleration));
        velocities.push_back(velocity);
        //used for plotting 
        positions.push_back(position);
        }
    while (position <= dist && velocity > 0){
        decelerate();
        position += velocity * dt;
        velocities.push_back(velocity);
        //used for plotting
        positions.push_back(position);
    }


    status = 2;
    return velocities;
}

void speedProfile::run(){
    if(status == 0){
        turn();
        velocities.clear();
        positions.clear();
        walk();
    }
    else if(status == 1){
        cout << "speedProfile running" << endl;
        return;
    }
    else{
        cout << "speedProfile finished" << endl;
        return;
    }
}

//return the wheel velocities for both wheel when going forward the distance between startPos and endPos
vector<vector<float>> speedProfile::walk(){
    vector<float> wheelSpeeds = getSpeedProfile(getDistance());
    for (int i = 0; i < wheelSpeeds.size(); i++){
        wheelVelocities.push_back({wheelSpeeds[i], wheelSpeeds[i]});
    }
    return wheelVelocities;
}

//return the wheel velocities for both wheel when turning by theta
vector<vector<float>> speedProfile::turn(){
    float theta = getTheta();
    theta = theta * (wheelBase / 2);
    vector<float> wheelSpeeds = getSpeedProfile(theta);
    if(theta < 0){
        for (int i = 0; i < wheelSpeeds.size(); i++){
            cout << "wheelSpeeds: " << wheelSpeeds[i] << endl;
            wheelVelocities.push_back({wheelSpeeds[i], -wheelSpeeds[i]});
        }
    }
    else{
        for (int i = 0; i < wheelSpeeds.size(); i++){
            wheelVelocities.push_back({-wheelSpeeds[i], wheelSpeeds[i]});
        }
    }
    return wheelVelocities;
}

vector<vector<float>> speedProfile::getWheelVelocities(){
    return this->wheelVelocities;
}

vector<float> speedProfile::getPositions(){
    return this->positions;
}

vector<float> speedProfile::getVelocities(){
    return this->velocities;
}

vector<vector<float>> speedProfile::forwardKinematics(vector<float> rightWheelVel, vector<float> leftWheelVel, float wheelRadius, float x, float y, float theta){
    float dt = 0.01;
    //positions = {x, y, theta}
    vector<vector<float>> positions;

    //initial position
    positions.push_back({x, y, theta});

    cout << "rightWheelVel.size(): " << rightWheelVel.size() << endl;

    for (int i = 0; i < rightWheelVel.size(); i++){
        float x = positions[i][0];
        float y = positions[i][1];
        float theta = positions[i][2];
        float rightWheel = rightWheelVel[i];
        float leftWheel = leftWheelVel[i];

        float v = (rightWheel + leftWheel) * (wheelRadius / 2);
        float w = (rightWheel - leftWheel) * (wheelRadius / wheelBase);

        float xNew = 0;
        float yNew = 0;
        float thetaNew = 0;

        if(abs(sin(theta)) < 0.0001){
            xNew = x + v * 1 * dt;
            yNew = y + v * 0 * dt;
            thetaNew = theta + w * dt;
        }
        else{
            xNew = x + v * cos(theta) * dt;
            yNew = y + v * sin(theta) * dt;
            thetaNew = theta + w * dt;
        }

        positions.push_back({xNew, yNew, thetaNew});
        //cout << "x: " << xNew << " y: " << yNew << " theta: " << thetaNew << endl;
    }

    return positions;
}

