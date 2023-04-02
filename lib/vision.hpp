#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void maien(Mat image, Scalar lower, Scalar upper);
int process_image(Scalar lower, Scalar upper, Mat image, vector<Point>* largest_contour_ptr, vector<Point>* second_largest_contour_ptr, vector<Point>* third_largest_contour_ptr = NULL);
float find_d(Mat img, vector<Point> contour, Point* contcent, Point* imgcent);

