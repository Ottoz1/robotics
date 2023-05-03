#include "speedProfile.hpp"

using namespace std;

int speedProfile::setNewGoal(float distance){
    if (status != 1){
        this->distance = distance;
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
    this->sampleTime = 0.1;
    this->maxVelocity = 10;    
    this->radius = 0.1;
    this->wheelBase = 0.2;

    //program variables
    this->breakDistance = 0;
    this->position = 0;
    this->velocity = 0;
    this->acceleration = 2;
    //distance between start and end position
    this->distance = sqrt(pow(endPos[0] - startPos[0], 2) + pow(endPos[1] - startPos[1], 2));
    this->angle = atan2(endPos[1] - startPos[1], endPos[0] - startPos[0]) + (direction * (M_PI / 180));

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

void speedProfile::turn(){
}

void speedProfile::run(){
    status = 1;
    while ((position + breakDistance)<= distance){
        if(velocity < maxVelocity){
            accelerate();
        } 
        else {
            holdVelocity();
        }
        position += velocity * sampleTime;
        breakDistance = pow(velocity, 2) / (2 * abs(acceleration));

        speeds.push_back(velocity);
        positions.push_back(position);
    }
    while (position <= distance && velocity > 0){
        decelerate();
        position += velocity * sampleTime;
        speeds.push_back(velocity);
        positions.push_back(position);
    }
    status = 2;
}

int speedProfile::getStatus(){
    return this->status;
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

std::vector<float> speedProfile::getWheelVelocities(float velocity){
    //wheel velocities = velocity / wheel radius
    std::vector<float> wheelVelocities;
    for (int i = 0; i < speeds.size(); i++){
        wheelVelocities.push_back(speeds[i] / radius);
    }
}