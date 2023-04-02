#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

void process_image(Mat image, Scalar lower, Scalar upper, int* predicted_number, vector<Point>* box_contour_ptr, vector<Point>* number_contour_ptr, vector<Point>* inner_number_contour_ptr, float* d);  // This function uses all functions underneath in order to extract the number and box from the image
void find_important_contours(Scalar lower, Scalar upper, Mat image, vector<Point>* largest_contour_ptr, vector<Point>* second_largest_contour_ptr, vector<Point>* third_largest_contour_ptr = NULL);
int biggest_within(vector<vector<Point>> contours, vector<Point> within);   // This function is used by "find_important_contours" to find the box, nummer within the box, and the inner_number within the number
float find_d(Mat image, vector<Point> contour);  // Finds the D value, a number between -1 or 1 in the X direction
int predict_number(vector<Point> number_contour, vector<Point> inner_number_contour);  // Predicts the number based on the ratio of the inner number to the outer number areas
void visualize_results(Mat image, vector<Point> largest_contour, vector<Point> number_contour, vector<Point> third_contour, int predicted_number);
