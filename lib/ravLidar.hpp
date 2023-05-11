#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

#define TCP_PORT 9887
#define BUFFER_SIZE 1024
#define MAX_CLIENTS 10

extern MatrixXf points;
extern int dataReady;
extern int lidarRunning;

int initLidar();
int stopLidar();
int sendToLidar(char* message);
int listenLidar();
