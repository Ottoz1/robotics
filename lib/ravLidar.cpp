#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <time.h>
#include "ravLidar.hpp"

int initLidar(){
    char buff[] = {0x10,0x00};
    if(sendToLidar(buff) != 0){
        printf("\ninitLidar Failed\n");
        return -1;
    }
    return 0;
}

int stopLidar(){
    char buff[] = {0x20,0x00};
    if(sendToLidar(buff) != 0){
        printf("\ninitLidar Failed\n");
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
    cli_addr.sin_addr.s_addr = htonl(TCP_IP);

    int s = connect(sock, (struct sockaddr*)&cli_addr, sizeof(cli_addr));
    if( s < 0 ){
        perror("Connection Failed");
        return -1; 
    }
    

    int size = *(&message + 1) - message; 
    send(sock, message, size, 0);
    close(sock);
    return 0;
}

int listen(){
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr;

    char sendBuff[BUFFER_SIZE];
    time_t ticks;


    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff));
    
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(9888);
    serv_addr.sin_addr.s_addr = htonl(TCP_IP);
    
    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, MAX_CLIENTS);

    /* 
     *
     */

    while(1){
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

        ticks = time(NULL);
		snprintf(sendBuff, sizeof(sendBuff), "%.24s\r\n", ctime(&ticks));
		write(connfd, sendBuff, strlen(sendBuff));

		close(connfd);
		sleep(1);
    }
}