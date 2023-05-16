#include "lib/ravCam.hpp"
#include "lib/cox.hpp"
#include "lib/motors.hpp"
#include "lib/ravLidar.hpp"
#include "lib/odometry.hpp"
#include "lib/positionUpdate.hpp"
#include "lib/speedProfile.hpp"
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
vector<float> start_pos = {190, 1230, 0};
vector<float> end_pos = {790, 480, M_PI/4};
vector<float> r_wheel_vel;
vector<float> l_wheel_vel;
int dataReady = 0;

int x;
int y;
int xs;
int ys;
int theta;

float max_v;

void vision_test()
{
    // Load the image
    Mat image = imread("../img/3.jpg");
    resize(image, image, Size(1280, 720));

    // Set the HSV color bounds for the filter
    Scalar lower = Scalar(41, 50, 0);
    Scalar upper = Scalar(115, 255, 240);

    // Process the image
    vector<Point> box_contour;  // Biggest contour in the image (suppose to be the box)
    vector<Point> number_contour;   // Biggest contour in the image within largest_contour (suppose to be the number)
    vector<Point> inner_number_contour;   // Biggest contour in the image within number_contour (zero will have a large contour here, 1 will not)
    int predicted_number;   // Predicted number on the box
    float d;    // How "in the middle" the box is (0 is in the middle, 1 or -1 is on the edge)
    process_image(image, lower, upper, &predicted_number, &box_contour, &number_contour, &inner_number_contour, &d);

    // Print stuff
    cout << "Predicted number: " << predicted_number << endl;
    cout << "d: " << d << endl;

    // Show the image with the contours and predicted number
    visualize_results(image, box_contour, number_contour, inner_number_contour, predicted_number);
}

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
    start_pose << 190, 1230, 0;  // Initial pose (x, y, theta)
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
    thread th1(listenLidar);
    thread th2(positionUpdater);
    
    chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    for (int i = 0; i < wheelVelocities.size(); i++){
        // Calculate the elapsed time
        chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsedTime = chrono::duration_cast<chrono::duration<double>>(currentTime - start);

        // Check if 10ms have passed
        if (elapsedTime.count() < 0.01){
            i--;
            continue;
        }
        
        // Update the start time for the next iteration
        start = chrono::high_resolution_clock::now();

        call_motors(wheelVelocities[i][0] * 900/max_v, wheelVelocities[i][1] * 900/max_v);
        VectorXf pos_current = get_odometry_pose();
        //cout << "iter: " << i << " of: " << wheelVelocities.size() << endl;   
        //cout << "time_elapsed: " << elapsedTime.count()*1000 << endl;   

    }

    th1.join();
    th2.join();
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

    // Create a camera feed
    VideoCapture cap(0);
    if (!cap.isOpened()){
        cout << "Error opening video stream" << endl;
        return -1;
    }

    init_motors();



    chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    while(1){
        // Calculate the elapsed time
        chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsedTime = chrono::duration_cast<chrono::duration<double>>(currentTime - start);

        // Check if 10ms have passed
        if (elapsedTime.count() < 0.01){
            continue;
        }
        
        // Update the start time for the next iteration
        start = chrono::high_resolution_clock::now();

        // Get the image
        Mat image;
        cap >> image;
        resize(image, image, Size(640, 480));
        rotate(image, image, ROTATE_180);   // Rotate the image 180 degrees

        // Set the HSV color bounds for the filter
        Scalar lower = Scalar(97, 56, 58);
        Scalar upper = Scalar(113, 218, 134);

        // Process the image
        vector<Point> box_contour;  // Biggest contour in the image (suppose to be the box)
        vector<Point> number_contour;   // Biggest contour in the image within largest_contour (suppose to be the number)
        vector<Point> inner_number_contour;   // Biggest contour in the image within number_contour (zero will have a large contour here, 1 will not)
        int predicted_number;   // Predicted number on the box
        float d = 0;    // How "in the middle" the box is (0 is in the middle, 1 or -1 is on the edge)
        process_image(image, lower, upper, &predicted_number, &box_contour, &number_contour, &inner_number_contour, &d);
        
        p1 = 3000;  // Forward speed P value
        p2 = 800;   // Turning speed P value
        int left_speed = p1 + d*p2;
        int right_speed = p1 - d*p2;
        cout << "left_speed: " << left_speed << " right_speed: " << right_speed << endl;
        call_motors(left_speed, right_speed); 

        if (box_contour.size() == 0 || number_contour.size() == 0 || inner_number_contour.size() == 0){
            imshow("Image", image);
            waitKey(25);
            continue;
        }

        // Visualise the results
        Mat vis_img = visualize_results(image, box_contour, number_contour, inner_number_contour, predicted_number);

        imshow("Image", vis_img);
        waitKey(25);
    }

    return 0;

    kalman_test();

    lidarRunning = 0;

    stopLidar();

    return 0;
}