#include "lib/ravCam.hpp"
#include "lib/cox.hpp"
#include "lib/ravLidar.hpp"
//#include "lib/motors.hpp"
#include <time.h>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <unistd.h>
#include <wiringPi.h>
#include <stdio.h>
#include <wiringPiSPI.h>
extern "C" {
    #include "lib/spi_com.h"
}

MatrixXf points(200,2);
int dataReady = 0;
int lidarRunning = 1;

VectorXf pose(3);

using namespace std;
using namespace Eigen;

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


MotorDataType MotorData;

static const int SPI_Channel = 1;

short Des_Speed = 0;
int SS = 0;
int Counter = 0;
void motor_test(){
    wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
	
	while(1){
		delay(50);
		Counter++;
		if(Counter > 40){
			Counter =0;
			switch(SS){
				case 0:
					Des_Speed = 3000;
					SS = 1;
				break;
				case 1:
					Des_Speed = -3000;
					SS = 2;
				break;
				case 2:
					Des_Speed = -3000;
					SS = 3;
				break;
				case 3:
					Des_Speed = 0;
					SS = 0;
				break;
			}
			printf("Speed_M1=%d Speed_M2=%d Enkoder_M1= %d Enkoder_M2 %d\n", MotorData.Act_Speed_M1,MotorData.Act_Speed_M2,MotorData.Encoder_M1,MotorData.Encoder_M2);
		}
		MotorData.Set_Speed_M1=Des_Speed;
		MotorData.Set_Speed_M2=150;
		Send_Read_Motor_Data(&MotorData);
	}
}

int main(int argc, char **argv){
    motor_test();
    return 0;
    InputParser input(argc, argv);
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