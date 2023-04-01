#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// ####### USEFUL FUNCTIONS #######
Mat find_countours(Scalar lower, Scalar upper, Mat image, vector<Point>* largest_contour_ptr, vector<Point>* second_largest_contour_ptr);
// ####### END OF USEFUL FUNCTIONS #######

int SCALER = 2;
Scalar HSV_LOW_BOUND = Scalar(41, 50, 0);
Scalar HSV_HIGH_BOUND = Scalar(115, 255, 240);

int main() {
    // Load an image from file and resize it (resize should be changed later)
    Mat original_img = imread("../img/3.jpg", IMREAD_COLOR);
    if (original_img.empty()) {
        cout << "Error loading image" << endl;
        return -1;
    }
    resize(original_img, original_img, Size(1280, 720));

    // Create a window to display the image
    namedWindow("Image", WINDOW_AUTOSIZE);

    // Filter the image and keep the largest contour
    vector<Point> largest_contour;
    vector<Point> number_contour;
    find_countours(HSV_LOW_BOUND, HSV_HIGH_BOUND, original_img, &largest_contour, &number_contour);

    // Draw the largest contour and the second largest contour within it
    drawContours(original_img, vector<vector<Point>>(1, largest_contour), 0, Scalar(0, 0, 255), 2);
    drawContours(original_img, vector<vector<Point>>(1, number_contour), 0, Scalar(0, 255, 0), 2);
    
    imshow("Image", original_img);
    waitKey(0);

    return 0;
}

// This function runs the images through the filter and returns the filtered image
Mat find_countours(Scalar lower, Scalar upper, Mat image, vector<Point>* largest_contour_ptr, vector<Point>* second_largest_contour_ptr) {
    Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, lower, upper, imgHSV);

    // Get contours and only keep the one with the largest area
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (contours.size() < 2){   // We need at least a box and a number on it, two contours
        printf("No contours found between the given HSV color bounds");
        return imgHSV;
    }

    // Find the largest contour (the box)
    int largest_contour_index = 0;
    int largest_area = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > largest_area) {
            largest_area = area;
            largest_contour_index = i;
        }
    }
    vector<Point> largest_contour = contours[largest_contour_index];

    // Find the second largest contour (the number) and make sure it's inside the box
    int second_largest_contour_index = 0;
    int second_largest_area = 0;
    for (int i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > second_largest_area && area < largest_area) {

            // Check if the contour is inside the box
            vector<Point> contour = contours[i];
            Point point_on_contour = contour[0];
            if (pointPolygonTest(largest_contour, point_on_contour, false) < 0) {
                continue;
            }

            second_largest_area = area;
            second_largest_contour_index = i;
        }
    }

    *largest_contour_ptr = contours[largest_contour_index];
    *second_largest_contour_ptr = contours[second_largest_contour_index];
    return imgHSV;
}