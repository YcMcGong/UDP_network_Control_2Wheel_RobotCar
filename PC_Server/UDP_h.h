#define PORT 1500
#define BUFSIZE 2048

#include<iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>	/* for fprintf */
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <unistd.h>

using namespace std;

struct hostent *hp;     /* host information */
struct sockaddr_in servaddr;    /* server address */
char *IP = "128.61.72.3";
int fs;                         /* our socket */
bool print_it = false;

int Send_setup()
{

    /* create a UDP socket */

    if ((fs = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("cannot create socket\n");
        return 0;
    }

    /* fill in the server's address and data */
    memset((char*)&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(IP);
    servaddr.sin_port = htons(PORT);
}

int Send_UDP(float* duty)
{
    /* send a message to the server */
    //cout<<"send: "<<duty[0]<<"\t"<<duty[1]<<endl;
    if (sendto(fs, duty, 2*sizeof(float), 0, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
	perror("sendto failed");
	return 0;
}
}

struct sockaddr_in myaddr;      /* our address */
struct sockaddr_in remaddr;     /* remote address */
socklen_t addrlen = sizeof(remaddr);            /* length of addresses */
int recvlen;                    /* # bytes received */
int fr;                         /* our socket */
float* recei;

int Receive_UDP_setup()
{
    /* create a UDP socket */

    if ((fr = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            perror("cannot create socket\n");
            return 0;
        }

    /* bind the socket to any valid IP address and a specific port */

    memset((char *)&myaddr, 0, sizeof(myaddr));
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(PORT);

    if (bind(fr, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
            perror("bind failed");
            return 0;
    }
}

float cur_rpm[3];
void* Receive_UDP_thread(void* i)
{
    Receive_UDP_setup(); //Setup Receiver

    /* now loop, receiving data and printing what we received */
    for (;;) {
        recvlen = recvfrom(fr, cur_rpm, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
        //printf("received %d bytes\n", recvlen);
        if((((int)cur_rpm[2])%50==0)&&print_it)
        cout<<"Receive   LEFT:"<<cur_rpm[0]<<"\t RIGHT:  "<<cur_rpm[1]<<endl;
    }
    /* never exits */   
}