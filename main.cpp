#include "lib/ravCam.hpp"
#include "lib/cox.hpp"
#include "lib/ravLidar.hpp"
#include <time.h>
#include <iostream>
#include <Eigen/Dense>

MatrixXd points(200,2);

using namespace std;

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

    // Define data types for start, stop, and duration variables
    clock_t start;
    clock_t end;
    double duration;

    start = clock();
    VectorXf transformation = cox_linefit(points, line_segments, 100);
    end = clock();

    points = transform_points(points, transformation);

    cout << "Transformation: \n" << transformation << endl;

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

int main(int argc, char **argv){

    InputParser input(argc, argv);
    if(input.cmdOptionExists("--start")){
        initLidar();
    }
    if(input.cmdOptionExists("--stop")){
        stopLidar();
    }
    if(input.cmdOptionExists("--receive")){
        listen();
    }
    const std::string &filename = input.getCmdOption("-f");
    if (!filename.empty()){
        // Do interesting things ...
    }
    return 0;
}