#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

extern Mat numMask;

void process_frame(Mat frame, Scalar lower, Scalar upper, vector<Rect>& boxes, vector<int>& identity_out, Scalar lower_purple, Scalar upper_purple, Scalar lower_yellow, Scalar upper_yellow, Scalar lower_darkBlue, Scalar upper_darkBlue);
void generate_mask(Mat hsv, Scalar lower, Scalar upper, Mat* mask, Mat* number_mask);
void extract_interesting_areas(vector<vector<Point>>& conts, vector<Vec4i>& hierarchy, Mat& frame, Mat& mask, vector<Rect>& interesting_boxes, vector<vector<Point>>& interesting_conts, vector<int>& identity);
vector<int> identify_numbers(const vector<Rect>& boxes, const Mat& mask, const Mat& number_mask, const vector<int>& identity, const Mat& frame);
Mat visualize_results(Mat frame, vector<Rect> boxes, vector<int> identity);
void detect_others(Mat hsv, Scalar lower_purple, Scalar upper_purple, Scalar lower_yellow, Scalar upper_yellow, Scalar lower_darkBlue, Scalar upper_darkBlue, vector<Rect>& boxes, vector<int>& identity_out);
float find_d(Mat img, Rect box);