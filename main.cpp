#include "lib/ravCam.hpp"
#include "lib/cox.hpp"
#include "lib/motors.hpp"
#include "lib/ravLidar.hpp"
#include "lib/odometry.hpp"
#include "lib/positionUpdate.hpp"
#include "lib/speedProfile.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include <time.h>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <unistd.h>
#include <stdio.h>
#include <chrono>


using namespace std;
using namespace Eigen;
using namespace cv;

VectorXf pose(3);

int new_pos_ready = 0;
int lidarRunning = 1;
MatrixXf points = MatrixXf::Zero(200,2);
VectorXf pos = VectorXf::Zero(3);
MatrixXf cov = MatrixXf::Zero(3, 3);
vector<vector<float>> positions_s;
vector<vector<float>> wheelVelocities;
vector<float> start_pos = {600, 1070, 0};
vector<float> end_pos = {790, 480, M_PI/2};
vector<float> r_wheel_vel;
vector<float> l_wheel_vel;
int dataReady = 0;

int x;
int y;
int xs;
int ys;
int theta;

float max_v;

class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }
        /// @author iain
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        /// @author iain
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector <std::string> tokens;
};

void init_robot(){

    float max_vel = max_v; // mm/10ms
    float dt = 0.01; // 10ms
    float acceleration = 10 * max_vel; // mm/10ms^2
    float dir = 0;
    
    end_pos[0] = x;
    end_pos[1] = y;
    start_pos[0] = xs;
    start_pos[1] = ys;
    start_pos[2] = theta;
    
    speedProfile sp(max_vel, dt, acceleration, start_pos, end_pos, dir);
    sp.run();
    wheelVelocities = sp.getWheelVelocities();
    
    for(int i = 0; i < wheelVelocities.size(); i++){
        r_wheel_vel.push_back(wheelVelocities[i][0]);
        l_wheel_vel.push_back(wheelVelocities[i][1]);
        cout << "i: " << i << " r: " << r_wheel_vel[i] << " l: " << l_wheel_vel[i] << endl;
    }

    positions_s = sp.forwardKinematics(r_wheel_vel, l_wheel_vel, WHEEL_RADIUS, start_pos[0], start_pos[1], dir);
    VectorXf start_pose(3);
    start_pose << start_pos[0], start_pos[1], start_pos[2];
    init_motors();
    init_odometry(start_pose);
    initLidar();
}

float errorTheta(VectorXf actual, vector<float> expected){
    float error = actual(2) - expected[2];
    return error;
}

void kalman_test(){
    init_robot();
    VectorXf endPos = VectorXf::Zero(3);
    endPos << end_pos[0], end_pos[1], end_pos[2];
    VectorXf startPos = VectorXf::Zero(3);
    startPos << (start_pos[0] + 200), start_pos[1], start_pos[2];
    thread th1(listenLidar);
    thread th2(positionUpdater);
    
    go_to(endPos);
    go_to(startPos);

    th1.join();
    th2.join();
}
/**/
void followBox(float d){
    int p1 = 3000;   // Forward speed P value
    int p2 = 250;   // Turning speed P value
    int left_speed = d*p2;
    int right_speed = -d*p2;
    p1 = 3000 - abs(d)*p1;
    left_speed += p1;
    right_speed += p1;
    call_motors(left_speed, right_speed);
}
/*
 * Behaviour of robot
 * Default behaviour: 
 */
 /*
int collectBoxes(){

    Eigen::VectorXf initialScanPos = VectorXf::Zero(3);
    initialScanPos << 1200, 1070, 0;
    Eigen::VectorXf currentPosition = get_odometry_pose();  // Get current position x, y, theta
    VectorXf startPos = VectorXf::Zero(3);
    startPos << (start_pos[0] + 200), start_pos[1], start_pos[2];

    init_robot();
    thread th1(listenLidar);
    thread th2(positionUpdater);

    // Create a camera feed
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout << "Error opening video stream" << endl;
        return -1;
    }

    init_motors();

    //start by going forward 400mm in x direction
    go_to(initialScanPos);

    //start main loop with default behaviour of spinning in place until a box is found with class 1
    chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    while(1){
        // Calculate the elapsed time
        chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsedTime = chrono::duration_cast<chrono::duration<double>>(currentTime - start);

        // Check if 10ms have passed
        if (elapsedTime.count() < 0.01){
            continue;
        }
        //
        //once a box is found with class 1, go to it
        // Get the image
        Mat image;
        cap >> image;
        resize(image, image, Size(640, 480));
        rotate(image, image, ROTATE_180);   // Rotate the image 180 degrees

        // Set the HSV color bounds for the filter
        Scalar lower = Scalar(70, 42, 0);
        Scalar upper = Scalar(139, 255, 255);

        // Process the frame
        vector<Rect> boxes;
        vector<int> identity;
        process_frame(image, lower, upper, boxes, identity);

        // Visualize the results
        Mat result = visualize_results(image, boxes, identity);
        imshow("Image", result);
        waitKey(25);


        //once a box is found with class -4 (box is collected) evaluate if the area of this box is large (indicating that two out of two boxes are collected)
        for(int i = 0; i < identify.count(); i++){
            if(identity[i] == -4){
                if(boxes[i].area() > 10000){
                    //if the area is large, go home
                    go_to(startPos);
                    goto exit;
                }
                else{
                    //if the area is small, continue collecting boxes
                }
            }
            if(identity[i] == 1){
                float d = find_d(image, boxes[i]);
                followBox(d);
            }
        }

        //if no box is found, spin in place
        if(boxes.size() == 0){
            call_motors(250, -250);
        }

    }

    exit:

    th1.join();
    th2.join();

    return;



    //exit loop

    //go home

    //reverse

}
*/


int main(int argc, char **argv){
    int p1 = 0;
    int p2 = 0;

    // Get input arguments and set motor speeds accordingly
    InputParser input(argc, argv);
    if(input.cmdOptionExists("-p1")){
        p1 = stoi(input.getCmdOption("-p1"));
    }
    if(input.cmdOptionExists("-p2")){
        p2 = stoi(input.getCmdOption("-p2"));
    }
    if(input.cmdOptionExists("-x")){
        x = stoi(input.getCmdOption("-x"));
    }
    if(input.cmdOptionExists("-y")){
        y = stoi(input.getCmdOption("-y"));
    }
    if(input.cmdOptionExists("-xs")){
        xs = stoi(input.getCmdOption("-xs"));
    }
    if(input.cmdOptionExists("-ys")){
        ys = stoi(input.getCmdOption("-ys"));
    }
    if(input.cmdOptionExists("-theta")){
        theta = stof(input.getCmdOption("-theta"));
    }
    if(input.cmdOptionExists("-h")){
        cout << "Usage: ./main [-p1 <p1>] [-p2 <p2>] [-x <x>] [-y <y>] [-xs <xs>] [-ys <ys>] [-theta <theta>] [-h]" << endl;
        cout << "p1: P value for forward speed" << endl;
        cout << "p2: P value for turning speed" << endl;
        cout << "x: x coordinate of end position" << endl;
        cout << "y: y coordinate of end position" << endl;
        cout << "xs: x coordinate of start position" << endl;
        cout << "ys: y coordinate of start position" << endl;
        cout << "theta: theta coordinate of start position" << endl;
        cout << "h: help" << endl;
        return 0;
    }

    kalman_test();
    //collectBoxes();

    lidarRunning = 0;

    stopLidar();

    return 0;
}