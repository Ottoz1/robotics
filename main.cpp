#include "lib/vision.hpp"
#include "lib/cox.hpp"

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

    plot(points, line_segments, "BEFORE_COX");
    VectorXf transformation = cox_linefit(points, line_segments, 100);
    points = transform_points(points, transformation);
    plot(points, line_segments, "AFTER_COX");

    cout << "Transformation: \n" << transformation << endl;
}

int main()
{
    //vision_test();
    cox_test();
    return 0;
}