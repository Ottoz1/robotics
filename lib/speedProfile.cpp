#include <time.h>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <unistd.h>

class speedProfile{
    //speed profile class constructor takes distance and sample rate, they are both constant
    const float distance;
    const float sampleRate;
    float velocity;
    

    public:
        //speed profile class constructor
        speedProfile(float distance, float sampleRate){
            this->distance = distance;
            this->sampleRate = sampleRate;
        }
        //speed profile class destructor
        ~speedProfile(){
            //nothing to do here
        }
        //speed profile class function to calculate the speed profile
        void calculateSpeedProfile();
        //speed profile class function to return the speed profile
        float* getSpeedProfile();
}