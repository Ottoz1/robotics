#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#define TCP_PORT 9887
#define BUFFER_SIZE 1024
#define MAX_CLIENTS 10
#define PI 3.14159265358979323846

extern MatrixXd points;

extern int dataReady;

int initLidar();
int stopLidar();
int sendToLidar(char* message);
int listen();
