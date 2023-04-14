#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

void cox_linefit(MatrixXf points, MatrixXf line_segments, int max_iter);
MatrixXf find_normals(MatrixXf lines);
MatrixXf assign_points_to_lines(MatrixXf points, MatrixXf line_segments, MatrixXf normals, MatrixXf* distances, MatrixXf* new_normals_ptr);
VectorXf get_signed_distance(MatrixXf points, MatrixXf targets, MatrixXf normals);

float point_segment_distance(VectorXf point, VectorXf line_segment);
MatrixXf generate_data();
MatrixXf arrayToMatrix(float* data, int numRows, int numCols);
void plot(MatrixXf points, MatrixXf lines, char* title);
MatrixXf generate_lines();
