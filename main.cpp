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

using namespace std;
using namespace Eigen;

VectorXf pose(3);

int new_pos_ready = 0;
int lidarRunning = 1;
MatrixXf points = MatrixXf::Zero(200,2);
VectorXf pos = VectorXf::Zero(3);
MatrixXf cov = MatrixXf::Zero(3, 3);
vector<vector<float>> positions_s;
vector<vector<float>> wheelVelocities;
Eigen::Vector2f start_pos(190, 1230);
Eigen::Vector2f end_pos(1020, 1230);
vector<float> r_wheel_vel;
vector<float> l_wheel_vel;
int dataReady = 0;

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

void cox_test()
{
    MatrixXf points = generate_data();  // Generate some random points
    MatrixXf line_segments = generate_lines();  // Generate lines

    // Multiply the points by 100 to make them more realistic
    points *= 2;

    // Define data types for start, stop, and duration variables
    clock_t start;
    clock_t end;
    double duration;

    MatrixXf cov = MatrixXf::Zero(3,3);
    start = clock();
    VectorXf transformation = cox_linefit(points, line_segments, 100, &cov);
    end = clock();

    points = transform_points(points, transformation);

    cout << "Transformation: \n" << transformation << endl;
    cout << "Covariance: \n" << cov << endl;

    duration = ((double)(end - start))/CLOCKS_PER_SEC;
    duration *= 1000;
    std::cout << "Execution time: " << duration << " microseconds." << std::endl;
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

void motor_test(short ml_speed_, short mr_speed_){
    init_motors();
    //pose << 0, 0, 0;  // Initial pose (x, y, theta)
    //init_odometry(pose);
	while(1){
		delay(100);
		call_motors(ml_speed_, mr_speed_);
        update_odometry_pose();
        //pose = get_odometry_pose();
        //printf("x=%f y=%f theta=%f\n",pose(0),pose(1),pose(2));
        MatrixXf cov = get_odometry_cov();
        printf("covariance: \n");
        cout << cov << endl;
        printf("________________________________\n");
	}
}

void init_robot(){
    float max_vel = max_v; // mm/10ms
    float dt = 0.01; // 10ms
    float acceleration = 10 * max_vel; // mm/10ms^2
    float dir = 0;
    speedProfile sp(max_vel, dt, acceleration, start_pos, end_pos, dir);
    sp.run();
    wheelVelocities = sp.getWheelVelocities();
    
    for(int i = 0; i < wheelVelocities.size(); i++){
        r_wheel_vel.push_back(wheelVelocities[i][0]);
        l_wheel_vel.push_back(wheelVelocities[i][1]);
    }

    positions_s = sp.forwardKinematics(r_wheel_vel, l_wheel_vel, WHEEL_RADIUS, start_pos(0), start_pos(1), dir);
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
    
    clock_t start = clock();
    for (int i = 0; i < wheelVelocities.size(); i++){
        // Calculate the elapsed time
        clock_t currentTime = clock();
        int elapsedTime = (currentTime - startTime) * 1000 / CLOCKS_PER_SEC;

        // Check if 10ms have passed
        if (elapsedTime < 10) {
            i--;
            continue;
        }
        
        // Update the start time for the next iteration
        start = clock();

        call_motors(wheelVelocities[i][0] * 3000/max_v, wheelVelocities[i][1] * 3000/max_v);
        VectorXf pos_current = get_odometry_pose();
        int ms = (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000);
        //cout << "iter: " << i << " of: " << wheelVelocities.size() << endl;        

    }

    th1.join();
    th2.join();
}

int main(int argc, char **argv){
    short right_speed;
    short left_speed;

    // Get input arguments and set motor speeds accordingly
    InputParser input(argc, argv);
    if(input.cmdOptionExists("-v")){
        max_v = stoi(input.getCmdOption("-v"));
    }
    else{
        max_v = 0;
    }

    if(input.cmdOptionExists("-r")){
        left_speed = stoi(input.getCmdOption("-r"));
    }
    else{
        left_speed = 0;
    }

    cout << "Left speed: " << left_speed << endl;
    cout << "Right speed: " << right_speed << endl;

    kalman_test();

    lidarRunning = 0;

    stopLidar();

    return 0;
}