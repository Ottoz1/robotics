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

// Define the color range
Scalar lower = Scalar(88, 73, 0);
Scalar upper = Scalar(107, 255, 255);

// Define obstacle color range
Scalar lower_purple = Scalar(132, 72, 0);
Scalar upper_purple = Scalar(161, 255, 255);
Scalar lower_yellow = Scalar(20, 93, 0);
Scalar upper_yellow = Scalar(30, 255, 255);

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
int turning = 0;

int blocks_taken = 0;

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
    cout << "start_pose: " << start_pose << endl;
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

void followBox(float d){
    cout << "Following box" << endl;
    int p1 = 3000;   // Forward speed P value
    int p2 = 400;   // Turning speed P value
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
 
int collectBoxes(){

    Eigen::VectorXf initialScanPos = VectorXf::Zero(3);
    initialScanPos << 1200, 1230, 0;
    Eigen::VectorXf currentPosition = get_odometry_pose();  // Get current position x, y, theta
    VectorXf startPos = VectorXf::Zero(3);
    startPos << 600, 1200, M_PI;
    init_robot();
    thread th1(listenLidar);
    thread th2(positionUpdater);
    thread th3(init_motors);

    start:

    // Create a camera feed
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout << "Error opening video stream" << endl;
        return -1;
    }

    //start by going forward 400mm in x direction
    go_to(initialScanPos);


    while(1){

        whileLoop:
        //
        //once a box is found with class 1, go to it
        // Get the image
        chrono :: high_resolution_clock :: time_point start = chrono::high_resolution_clock::now();
        Mat image;
        cap >> image;
        resize(image, image, Size(640, 480));
        rotate(image, image, ROTATE_180);   // Rotate the image 180 degrees

        // Process the frame
        vector<Rect> boxes;
        vector<int> identity;
        process_frame(image, lower, upper, boxes, identity, lower_purple, upper_purple, lower_yellow, upper_yellow);

        Mat results = visualize_results(image, boxes, identity);
        imshow("Results", results);
        waitKey(1);

        chrono :: high_resolution_clock :: time_point end = chrono::high_resolution_clock::now();

        cout << "Time process fram: " << chrono::duration_cast<chrono::duration<double>>(end - start).count() << endl;

        //Mat results = visualize_results(image, boxes, identity);
        //imshow("Results", results);
        //waitKey(1);


        //once a box is found with class -4 (box is collected) evaluate if the area of this box is large (indicating that two out of two boxes are collected)
        for(int i = 0; i < identity.size(); i++){
            if(identity[i] == -4){
                if(blocks_taken == 1)
                {
                    VectorXf temp_point = VectorXf::Zero(3);
                    temp_point << startPos(0)-300, startPos(1), startPos(2);
                    go_forward(200);
                    blocks_taken++;
                    go_to(startPos);
                    // Turn towards temp_point 
                    go_to(temp_point);
                    goto exit;
                }

                if(blocks_taken == 0){
                    cv::destroyAllWindows();
                    go_forward(200);
                    go_to(initialScanPos);
                    blocks_taken++;
                    goto start;
                }
            }
            if(identity[i] == 1){

                float d = find_d(image, boxes[i]);
                cout << "d: " << d << endl;
                followBox(d);
                goto whileLoop;
            }
        }

        //if no box is found, spin in place
        call_motors(250, -250);

        

    }

    exit:

    th1.join();
    th2.join();

    return 1;



    //exit loop

    //go home

    //reverse

}


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

    //kalman_test();
    collectBoxes();

    lidarRunning = 0;

    stopLidar();

    return 0;
}