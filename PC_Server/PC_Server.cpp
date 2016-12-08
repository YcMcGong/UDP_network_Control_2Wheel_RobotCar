#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <string>

using namespace std;

#define resolution_encoder 1

#include "UDP_h.h"
#define LEFT 0
#define RIGHT 1

volatile int encoder0Pos = 0;

int sampling_time = 10; //in ms
float sampling_freq = 100; //in Hz
int ref_rpm[2] = {40,40}; //update this
int max_rpm = 34; //calculate max motor rpm at max pwm value
int last_error[2];
float I[2];
float Kp_l = 0.22, Ki_l = 0.45, Kd_l = 0.00005;
float Kp_r = 0.22, Ki_r = 0.45, Kd_r = 0.00005;
//float Kp_r = 0.22, Ki_r = 1, Kd_r = 0.00005;
float  desired_duty_cycle[2];
float empty_duty_cycle[2]={0,0};
void doEncoderA(void);
void PID_calc();

#define NS_PER_SEC	1000000000

#define PWM_FREQ		490
#define PWM_FREQ_MULT	5
#define DUTY_CYCLE		49

#define DEF_PRIORITY	50

/* wiringPi 3 -> GPIO 22 */
#define DEF_PWM_PIN		3

extern int clock_nanosleep(clockid_t __clock_id, int __flags,
		__const struct timespec* __req, struct timespec* __rem);


/* weird timespec normalization function */
// void tsnorm(struct timespec *ts);


// void tsnorm(struct timespec *ts) {
// 	while (ts->tv_nsec >= NS_PER_SEC) {
// 		ts->tv_nsec -= NS_PER_SEC;
// 		ts->tv_sec++;
// 	}
// }
float run_time;
bool flow = false;
void* PID_Motor(void* i)
{
    while (1)
    {
	   if(flow==true) PID_calc();
    }
}


// void* runtime_check(void*i)
// {
//     while(1)
//     {
//         if(run_time>0){
//             usleep(100000);
//             run_time = run_time - 0.1;
//         }
//         else {flow=false; usleep(100000);}
//     }
// }


// void* receive_check(void* i)
// {
//     float check_stamp = last_stamp; 
//     while(1)
//     {
//         sleep(1);
//         if(check_stamp != last_stamp){
//             check_stamp = last_stamp;
//         }
//         else flow = false;
//     }
// }

int main()
{
    //Essentials
    double period=10000000;
    double c = 0;
    pthread_t pt1;
    pthread_create(&pt1, 0 ,Receive_UDP_thread, (void*)1);
    Send_setup();
    pthread_t pt2;
    pthread_create(&pt2, 0 ,PID_Motor, (void*)2);
    // pthread_t pt3;
    // pthread_create(&pt3, 0 ,runtime_check, (void*)3);

    //Operations after here
    float num[3];
    char com[3];
    char control;
    while (1)
    {

        cout<<"Ready to Receive Input L and R "<<endl;
        cin >> com[0] >> num[0] >> com[1] >> num[1];
        // if(com[0] == 'L') {
        //     if(com[1]=='R') {ref_rpm[LEFT]=num[0]; ref_rpm[RIGHT]=num[1];print_it = true;}
        //     else if(com[1]='r') {ref_rpm[LEFT]=num[0]; ref_rpm[RIGHT]=(num[0]/num[1]);print_it = true;}
        // }
        // else if(com[0] == 'R'){
        //     if(com[1]=='L') {ref_rpm[RIGHT]=num[0]; ref_rpm[LEFT]=num[1];print_it = true;}
        //     else if(com[1]='r') {ref_rpm[RIGHT]=num[0]; ref_rpm[LEFT]=(num[0]/num[1]);print_it = true;}
        // }
        // else;

        ref_rpm[LEFT]=num[0]; ref_rpm[RIGHT]=num[1];flow = true;print_it = true; //run_time = num[2];

        cin>>control;
        if(control == 'r') print_it = false;
        else {ref_rpm[RIGHT]=0; ref_rpm[LEFT]=0;print_it = false;}

    }
  return 0;
  
}

float last_stamp=-1;
float error[2];
float D[2];

void PID_calc()
{
    while(cur_rpm[2] == last_stamp) usleep(500);

    /* Motor 1*/
    if (cur_rpm[LEFT] <0)
	cur_rpm[LEFT] = -cur_rpm[LEFT]; 
	  
	error[LEFT] = (ref_rpm[LEFT] - cur_rpm[LEFT]) *100 /max_rpm;
    I[LEFT] += error[LEFT]*(1/sampling_freq);
    D[LEFT] = (error[LEFT] - last_error[LEFT])*sampling_freq;
    desired_duty_cycle[LEFT] = 45 + Kp_l*error[LEFT] + Ki_l*I[LEFT] + Kd_l*D[LEFT];
         
	if (desired_duty_cycle[LEFT] >= 100)
        { 
          desired_duty_cycle[LEFT] = 100;
        }
        else if(desired_duty_cycle[LEFT] <8)
        {       
                desired_duty_cycle[LEFT] =  8;
        }

    last_error[LEFT] = error[LEFT];


    /*  Motor 2 */
    if (cur_rpm[RIGHT] <0)
	cur_rpm[RIGHT] = -cur_rpm[RIGHT]; 
	  
	error[RIGHT] = (ref_rpm[RIGHT] - cur_rpm[RIGHT]) *100 /max_rpm;
    I[RIGHT] += error[RIGHT]*(1/sampling_freq);
    D[RIGHT] = (error[RIGHT] - last_error[RIGHT])*sampling_freq;
    desired_duty_cycle[RIGHT] = 45 + Kp_r*error[RIGHT] + Ki_r*I[RIGHT] + Kd_r*D[RIGHT];
         
	if (desired_duty_cycle[RIGHT] >= 100)
        { 
          desired_duty_cycle[RIGHT] = 100;
        }
        else if(desired_duty_cycle[RIGHT] <8)
        {       
                desired_duty_cycle[RIGHT] =  8;
        }

    last_error[RIGHT] = error[RIGHT];

    //Send the duty cycle
    Send_UDP(desired_duty_cycle);
    //Send_UDP(empty_duty_cycle);
    last_stamp = cur_rpm[2];

}