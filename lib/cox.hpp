#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

void cox_linefit();
MatrixXf generate_data();
MatrixXf arrayToMatrix(float* data, int numRows, int numCols);
void plot(MatrixXf points);
