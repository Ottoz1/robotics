#include <Eigen/Dense>

using namespace Eigen;

#define TCP_PORT 9887
#define BUFFER_SIZE 1024
#define MAX_CLIENTS 10

extern MatrixXd points;

int initLidar();
int stopLidar();
int sendToLidar(char* message);
int listen();
