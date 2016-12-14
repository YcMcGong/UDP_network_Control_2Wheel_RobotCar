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
char *IP = "128.61.77.128";
int fs;                         /* our socket */


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

int Send_UDP(float *rpm)
{
    /* send a message to the server */
    //char my_message[15]; 
    //sprintf (my_message, "%f", duty);
    //printf("Sending message of rpm[0]: %f %f %f\n", rpm[0], rpm[1], rpm[2]);
    if (sendto(fs, rpm, 3*sizeof(float), 0, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
	perror("sendto failed");
	return 0;
}
}

struct sockaddr_in myaddr;      /* our address */
struct sockaddr_in remaddr;     /* remote address */
socklen_t addrlen = sizeof(remaddr);            /* length of addresses */
int recvlen;                    /* # bytes received */
int fr;                         /* our socket */
float buf[BUFSIZE];     /* receive buffer */

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




struct motor_struct {
	int n_pin;			/* PWM pin */
	int dir_pin;		/* directional pin (can be NULL) */
	float duty_cycle;		/* duty cycle 0-100 */
	int period;			/* period in ns	*/
	int debug;			/* whether to print stuff */
	struct timespec *t;	/* timer structure for PWM */
};
struct motor_struct ma;



void* Receive_UDP_thread(void* i)
{
    Receive_UDP_setup(); //Setup Receiver

    /* now loop, receiving data and printing what we received */
    for (;;) {
        //printf("waiting on port %d\n", PORT);
        recvlen = recvfrom(fr, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
        //printf("received %d bytes\n", recvlen);
        if (recvlen > 0) {
                buf[recvlen] = 0;
                //printf("received message of buf[0]: %f %f\n", buf[0], buf[1]);

                ma.duty_cycle = buf[0];              
                
                pwmWrite(1, (int)(buf[1]*10.23));
            }
    }
    /* never exits */   
}

// int main()
// {
//     pthread_t pt;
//     pthread_create(&pt, 0 ,Receive_UDP_thread, (void*)1);
//     Send_setup();
//     while(1)
//     {
//         char *my_message = "It is working";
//         Send_UDP(my_message);
//         cout<<"sent"<<endl;
//         sleep(3);
//     }
//     return 0;
// }
