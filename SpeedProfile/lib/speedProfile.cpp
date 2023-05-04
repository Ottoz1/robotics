#include "speedProfile.hpp"

using namespace std;

int speedProfile::setNewGoal(Eigen::Vector2f goal){
    if (status != 1){
        this->endPos = goal;
        this->distance = sqrt(pow(endPos[0] - startPos[0], 2) + pow(endPos[1] - startPos[1], 2));
        this->angle = atan2(endPos[1],endPos[0]);
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
    //wheel radius
    this->radius = 1;
    //distance between wheels
    this->wheelBase = 0.2;

    //program variables
    this->velocity = 0;
    this->acceleration = 1;
    //distance between start and end position
    this->distance = sqrt(pow(endPos[0] - startPos[0], 2) + pow(endPos[1] - startPos[1], 2));
    this->angle = atan2(endPos[1],endPos[0]);
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

    std::vector<std::vector<float>> turningVelocities;
    std::vector<std::vector<float>> straightVelocities;
    cout << "angle: " << angle << endl;
    getSpeedProfile(angle);

    turningVelocities = getWheelTurnVelocities();


    getSpeedProfile(distance);
    straightVelocities = getWheelVelocities();

    for (int i = 0; i < turningVelocities.size(); i++){
        wheelVelocities.push_back(turningVelocities[i]);
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
        positions.push_back(position);
    }
    while (position <= dist && velocity > 0){
        decelerate();
        position += velocity * sampleTime;
        speeds.push_back(velocity);
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
    // wheelVelocities = [leftWheel, rightWheel]
    std::vector<std::vector<float>> wheelVelocities;
    for (int i = 0; i < speeds.size(); i++){
        wheelVelocities.push_back({speeds[i] / radius, speeds[i] / radius});
    }
    return wheelVelocities;
}

std::vector<std::vector<float>> speedProfile::getWheelTurnVelocities(){
    // wheelVelocities = [leftWheel, rightWheel]
    std::vector<std::vector<float>> wheelVelocities;
    //if the angle in radians is negative, turn left
    if(angle > 0){
        for (int i = 0; i < speeds.size(); i++){
            wheelVelocities.push_back({speeds[i] / radius, -speeds[i] / radius});
        }
    }
    else{
        for (int i = 0; i < speeds.size(); i++){
            wheelVelocities.push_back({-speeds[i] / radius, speeds[i] / radius});
        }
    }
    return wheelVelocities;
}