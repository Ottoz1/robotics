#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <time.h>
#include <math.h>
#include "ravLidar.hpp"
#include "cox.hpp"

int iter = 0;

int initLidar(){
    char buff[] = {0x10,0x00};
    printf("\nInitialzing");
    if(sendToLidar(buff) != 0){
        return -1;
    }
    return 0;
}

int stopLidar(){
    char buff[] = {0x20,0x00};
    printf("\nStopping");
    if(sendToLidar(buff) != 0){
        return -1;
    }
    return 0;
}

int sendToLidar(char* message){
    int sock;
    int status;
    struct sockaddr_in cli_addr;
    sock = socket(AF_INET, SOCK_STREAM, 0);

    if(sock < 0){
        perror("socket failed");
        return -1;
    }
        
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_port = htons(TCP_PORT);
 
    if (inet_pton(AF_INET, "127.0.0.1", &cli_addr.sin_addr) <= 0) {  
        printf("\nInvalid address/ Address not supported \n");  
        return -1;  
    }  

    int s = connect(sock, (struct sockaddr*)&cli_addr, sizeof(cli_addr));
    if( s < 0 ){
        perror("Connection Failed");
        return -1; 
    }
    

    int size = *(&message + 1) - message;
    send(sock, message, 2, 0);
    close(sock);
    return 0;
}

int listen(){
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr;

    lidarRunning = 1;

    char sendBuff[BUFFER_SIZE];
    time_t ticks;

    char header[5] = { 0 };
    char buffer[4096] = { 0 };

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff));
    
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(9888);
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {  
        printf("\nInvalid address/ Address not supported \n");  
        return -1;  
    }  
    
    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    /* 
     *
     */
    connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

    printf("lidar running: %d", lidarRunning);

    while(lidarRunning == 1){
        printf("inside of love\n");
        int header_size = read(connfd, header, 5);
        if(header_size != 5){
            printf("inside of break\n");
            break;
        }
        printf("rad 103>:D\n");
        if((int)header[0] == 165){
            printf("inside if\n");
            int data_size = ((int)(header[2])<<16) + ((int)(header[3])<<8) + ((int)(header[4]));
            read(connfd, buffer, data_size);
            int quality = (int)(buffer[0])>>2;
            int angle = (((int)(buffer[1])>>1) + ((int)(buffer[2])<<8))>>7;
            int theta = angle * (PI / 180);
            int distance = (((int)(buffer[3])) + ((int)(buffer[4])<<8))>>2;
            printf("radius: %d, angle: %d", distance, angle);

            if(dataReady == 0){
                points(iter,0) = 3;//distance * cos(theta);
                points(iter,1) = 2;//distance * sin(theta);
                iter++;

                if(iter >= 200){
                    iter = 0;
                    dataReady = 1;
                }
            }

        }
    }
    close(connfd);
    return 0;
}