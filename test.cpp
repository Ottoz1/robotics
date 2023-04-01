#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

// ####### USEFUL FUNCTIONS #######
Mat find_countours(Scalar lower, Scalar upper, Mat image, vector<Point>* largest_contour_ptr);
Mat crop_bounding_box(Mat filtered_image);
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
    Mat binary_img = find_countours(HSV_LOW_BOUND, HSV_HIGH_BOUND, original_img, &largest_contour);
    Mat cropped_binary_img = original_img(boundingRect(largest_contour));


    // Draw the largest contour on the image
    drawContours(original_img, vector<vector<Point>>(1, largest_contour), 0, Scalar(0, 0, 255), 2);

    // Display the image with the highlighted contour
    imshow("Image", original_img);
    waitKey(0);

    return 0;
}

// This function runs the images through the filter and returns the filtered image
Mat find_countours(Scalar lower, Scalar upper, Mat image, vector<Point>* largest_contour_ptr) {
    Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, lower, upper, imgHSV);

    // Get contours and only keep the one with the largest area
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    if (contours.size() == 0){
        printf("No contours found between the given HSV color bounds");
    }

    // Remove all contours except the largest one
    int largest_contour_index = 0;
    int largest_area = 0;
    for (int i = 0; i < contours.size(); i++){
        double a = contourArea(contours[i], false);
        if (a > largest_area){
            largest_area = a;
            largest_contour_index = i;
        }
    }

    // Remove all other contours from the image
    for (int i = 0; i < contours.size(); i++){
        if (i != largest_contour_index){
            drawContours(imgHSV, contours, i, Scalar(0, 0, 0), -1);
        }
    }

    *largest_contour_ptr = contours[largest_contour_index];
    return imgHSV;
}