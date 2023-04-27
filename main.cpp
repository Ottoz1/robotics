#include "lib/ravCam.hpp"
#include "lib/cox.hpp"
#include "lib/ravLidar.hpp"
#include <time.h>

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

int main(int argc, char** argv)
{
    //vision_test();
    //cox_test();
    int start = 0;
    int stop = 0;
    int rec = 0;
    int s;
    for (int i = 0; i < argc; i++){
        if (strcmp(argv[i], "--start") == 0){
            start = 1;
        }
        else if (strcmp(argv[i], "--stop") == 0){
            stop = 1;
        }
        else if (strcmp(argv[i], "--receive") == 0){
            rec = 1;
        }
    }

    if(start = 1){
        s = initLidar();
        if(s != 0){
            perror("Lidar Init failed");
        }
        printf("\n Initialized");
    }
    if(stop = 1){
        s = stopLidar();
        if(s != 0){
            perror("Lidar Stop failed");
        }
    }
    if(rec = 1){
        listen();
    }
    return 0;
}