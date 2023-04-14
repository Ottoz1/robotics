#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

void cox_linefit(MatrixXf points, MatrixXf line_segments);
MatrixXf generate_data();
MatrixXf arrayToMatrix(float* data, int numRows, int numCols);
void plot(MatrixXf points, MatrixXf lines, char* title);
MatrixXf generate_lines();
