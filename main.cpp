#include "lib/ravCam.hpp"
#include "lib/cox.hpp"
#include "lib/motors.hpp"
#include "lib/ravLidar.hpp"
#include "lib/odometry.hpp"
#include "lib/positionUpdate.hpp"
#include <time.h>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <unistd.h>
#include <stdio.h>

using namespace std;
using namespace Eigen;

VectorXf pose(3);

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
        printf("__________________________\n");
	}
}

void init_robot(){
    VectorXf start_pose(3);
    start_pose << 190, 1220, 0;  // Initial pose (x, y, theta)

    //--Points used to transfer points between threads--//
    //Initialized before threads are started
    MatrixXf points(200,2);
    //--Pos used to transfer points between threads--//
    //Initialized before threads are started
    VectorXf pos(3) = VectorXf::Zero(3);
    MatrixXf cov(3, 3) = MatrixXf::Zero(3, 3);
    int new_pos_ready = 0;

    init_motors();
    init_odometry(start_pose);
    initLidar();
}

void kalman_test(short ml_speed_, short mr_speed_){
    init_robot();
    thread th1(listenLidar);
    thread th2(positionUpdater);
    MatrixXf line_segments = generate_lines();  // Generate environment lines
    
    time_t start = time(NULL); 
    while (1){
        time_t end = time(NULL);
        double elapsed_seconds = difftime(end, start);

        if (elapsed_seconds >= 100.0) {
            lidarRunning = 0;
            break;
        }

        call_motors(ml_speed_, mr_speed_);

        if(new_pos_ready){

            new_pos_ready = 0;
        }

    }
}

int main(int argc, char **argv){
    short right_speed;
    short left_speed;

    // Get input arguments and set motor speeds accordingly
    InputParser input(argc, argv);
    if(input.cmdOptionExists("-l")){
        right_speed = stoi(input.getCmdOption("-l"));
    }
    else{
        right_speed = 0;
    }

    if(input.cmdOptionExists("-r")){
        left_speed = stoi(input.getCmdOption("-r"));
    }
    else{
        left_speed = 0;
    }

    cout << "Left speed: " << left_speed << endl;
    cout << "Right speed: " << right_speed << endl;

    kalman_test(left_speed, right_speed);
    return 0;

    MatrixXf line_segments = generate_lines();
    thread th1(listenLidar);
    initLidar();
    MatrixXf cart;

    pose << 445, 280, 0;   // Initial pose (x, y, theta)

    sleep(2);

    time_t start = time(NULL); 
    while (1){
        time_t end = time(NULL);
        double elapsed_seconds = difftime(end, start);

        if (elapsed_seconds >= 100.0) {
            lidarRunning = 0;
            break;
        }
        
        if(dataReady = 1){
            printf("\ncox time\n");
            printf("|---------------|\n");

            cart = polar_to_cart(points);
            cart = transform_points(cart, pose);    // Laser to world frame

            MatrixXf cov = MatrixXf::Zero(3,3);
            VectorXf transformation = cox_linefit(cart, line_segments, 100, &cov);
            cout << transformation;

            cart = transform_points(cart, transformation);  // Transform points to align with lines
            pose(0) += transformation(0);
            pose(1) += transformation(1);
            pose(2) += transformation(2);

            plot(cart);
            printf("\n|---------------|");

            dataReady=0;
        }
    }

    lidarRunning = 0;

    th1.join();

    stopLidar();

    return 0;
}