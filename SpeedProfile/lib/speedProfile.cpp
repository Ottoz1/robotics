#include "speedProfile.hpp"

using namespace std;

 /*
 / Speed profile for a differential drive robot
*/

int speedProfile::setNewGoal(Eigen::Vector2f goal){
    if (status != 1){
        this->endPos = goal;
        this->distance = sqrt(pow(endPos[0] - startPos[0], 2) + pow(endPos[1] - startPos[1], 2));
        this->angle = atan2(endPos[1] - startPos[1], endPos[0] - startPos[0]);
        return 0;
    }
    else{
        return 1;
    }
}

speedProfile::speedProfile(Eigen::Vector2f startPos, Eigen::Vector2f endPos, float direction){
    //user specified variables
    this->startPos = startPos;
    this->endPos = endPos;
    this->direction = direction;

    //robot constants
    this->sampleTime = 0.01;
    this->maxVelocity = 10;   
    this->motorValue = 1000; 
    //wheel radius
    this->radius = 1;
    //distance between wheels
    this->wheelBase = 2;

    //program variables
    this->velocity = 0;
    this->acceleration = 10;
    //distance between start and end position
    this->distance = sqrt(pow(endPos[0] - startPos[0], 2) + pow(endPos[1] - startPos[1], 2));
    this->angle = atan2(endPos[1] - startPos[1], endPos[0] - startPos[0]);
    positions.clear();
    speeds.clear();
}

speedProfile::~speedProfile(){
    // destructor
}

void speedProfile::accelerate(){
    acceleration = std::abs(acceleration);
    velocity += acceleration * sampleTime;
    
    return;
}
void speedProfile::holdVelocity(){
    velocity = maxVelocity;
    return;
}
void speedProfile::decelerate(){
    acceleration = -std::abs(acceleration);
    velocity += acceleration * sampleTime;
    return;
}

void speedProfile::run(){
    //get speed profile for turning
    //convert to wheel velocities
    //append to wheelVelocities
    //get speed profile for straight line
    //convert to wheel velocities
    //append to wheelVelocities

    assert(positions.size() == 0);
    assert(speeds.size() == 0);
    assert(profiles.size() == 0);

    std::vector<std::vector<float>> turningVelocities;
    std::vector<std::vector<float>> straightVelocities;
    std::vector<std::vector<float>> turningProfiles;
    std::vector<std::vector<float>> straightProfiles;
    getSpeedProfile(angle * (wheelBase / 2));

    turningVelocities = calcWheelTurnVelocities();
    turningProfiles = calcWheelTurnProfiles();

    positions.clear();
    speeds.clear();
    profiles.clear();

    assert(positions.size() == 0);
    assert(speeds.size() == 0);

    getSpeedProfile(distance);
    straightVelocities = calcWheelVelocities();
    straightProfiles = calcWheelProfiles();

    for (int i = 0; i < turningVelocities.size(); i++){
        wheelVelocities.push_back(turningVelocities[i]);
    }
    for (int i = 0; i < straightVelocities.size(); i++){
        wheelVelocities.push_back(straightVelocities[i]);
    }
}

void speedProfile::getSpeedProfile(float dist){
    float position = 0;
    double breakDistance = 0;
    status = 1;
    while ((position + breakDistance)<= dist){
        if(velocity < maxVelocity){
            accelerate();
        } 
        else {
            holdVelocity();
        }
        position += velocity * sampleTime;
        breakDistance = pow(velocity, 2) / (2 * abs(acceleration));

        speeds.push_back(velocity);
        profiles.push_back(motorValue);
        positions.push_back(position);
    }
    while (position <= dist && velocity > 0){
        decelerate();
        position += velocity * sampleTime;
        speeds.push_back(velocity);
        profiles.push_back(0);
        positions.push_back(position);
    }
    status = 2;
    return;
}
int speedProfile::getStatus(){
    return this->status;
}
int speedProfile::getAngle(){
    return this->angle;
}
int speedProfile::getSampleTime(){
    return this->sampleTime;
}

std::vector<float> speedProfile::getSpeeds(){
    return speeds;
}

std::vector<float> speedProfile::getPositions(){
    return positions;
}

std::vector<std::vector<float>> speedProfile::getWheelVelocities(){
    return this->wheelVelocities;
}


std::vector<std::vector<float>> speedProfile::calcWheelVelocities(){
    // wheelVelocities = [leftWheel, rightWheel]
    std::vector<std::vector<float>> wheelVel;
    for (int i = 0; i < speeds.size(); i++){
        wheelVel.push_back({speeds[i] / radius, speeds[i] / radius});
    }
    return wheelVel;
}

std::vector<std::vector<float>> speedProfile::calcWheelTurnVelocities(){
    // wheelVelocities = [leftWheel, rightWheel]
    std::vector<std::vector<float>> wheelVel;
    //if the angle in radians is negative, turn left
    if(angle > 0){
        for (int i = 0; i < speeds.size(); i++){
            wheelVel.push_back({speeds[i] / radius, -speeds[i] / radius});
        }
    }
    else{
        for (int i = 0; i < speeds.size(); i++){
            wheelVel.push_back({-speeds[i] / radius, speeds[i] / radius});
        }
    }
    return wheelVel;
}

std::vector<std::vector<float>> speedProfile::calcWheelProfiles(){
    // wheelVelocities = [leftWheel, rightWheel]
    std::vector<std::vector<float>> wheelProf;
    for (int i = 0; i < profiles.size(); i++){
        wheelProf.push_back({profiles[i], profiles[i]});
    }
    return wheelProf;
}

std::vector<std::vector<float>> speedProfile::calcWheelTurnProfiles(){
    // wheelVelocities = [leftWheel, rightWheel]
    std::vector<std::vector<float>> wheelProf;
    if(angle > 0){
        for (int i = 0; i < speeds.size(); i++){
            wheelProf.push_back({profiles[i], -profiles[i]});
        }
    }
    else{
        for (int i = 0; i < speeds.size(); i++){
            wheelProf.push_back({-profiles[i], profiles[i]});
        }
    }
    return wheelProf;
}


