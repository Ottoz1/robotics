#define TCP_IP 2130706433 //"127.0.0.1"
#define TCP_PORT 9887
#define BUFFER_SIZE 1024
#define MAX_CLIENTS 10

int initLidar();
int stopLidar();
int sendToLidar(char* message);
int listen();