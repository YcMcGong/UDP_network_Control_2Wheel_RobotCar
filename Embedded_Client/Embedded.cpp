#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>
#include <iostream>
#include <wiringPi.h>
#include <pthread.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "UDP_h.h"

using namespace std;

#define encoder0PinA 23	// First pin of quadrature encoder and motor 1
#define encoder0PinB 22	// Second pin of quadrature encoder and  motor 1
#define motor_dir 5		// Motor direction pin of motor 1
#define motor_pwm 27	// PWM pin for motor 1
#define encoder1PinA 29	// First pin of quadrature encoder and motor 2
#define encoder1PinB 28	// Second pin of quadrature encoder and motor 2
#define motor_dir1 26	// Motor direction pin of motor 2
#define motor_pwm1 1	// PWM pin for motor 2
#define resolution_encoder 1

#define PORT 1500
#define BUFSIZE 2048
#define LEFT 0
#define RIGHT 1


volatile int encoder0Pos = 0;
volatile int encoder1Pos = 0;


int sampling_time = 10; //in ms
float sampling_freq = 100; //in Hz
int counts_per_rotation = 360; //update this
int ref_rpm = 30; //update this
int max_rpm = 34; //calculate max motor rpm at max pwm value
int max_pwm = 255; //depends on pwm resolution
int least_pwm_for_motor = 100; //calculate min pwm required to run motor
int last_error = 0;
float I = 0;
int last_A = 0;
float Kp = 0.22, Ki = 1, Kd = 0.0005;
double desired_rpm = 0;
double  desired_duty_cycle = 0;

void doEncoderA(void);
void doEncoderB(void);
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
void tsnorm(struct timespec *ts);

/* change state and write output */
void out(int n_pin, int* state);

void tsnorm(struct timespec *ts) {
	while (ts->tv_nsec >= NS_PER_SEC) {
		ts->tv_nsec -= NS_PER_SEC;
		ts->tv_sec++;
	}
}

void out(int n_pin, int* state) {
	*state = !*state;
	digitalWrite(n_pin, *state);
}
 
void* PWM_thread(void* v)
{
    ma.n_pin=motor_pwm;
    ma.dir_pin=motor_dir;
    ma.duty_cycle = 0;
    ma.period=2000000;
    struct timespec t_pwm;
    clock_gettime(0, &t_pwm);
    ma.t = &t_pwm;
    int state = 0;
    while (1) {
		clock_nanosleep(0, TIMER_ABSTIME, ma.t, NULL);
		/* if duty_cycle == 100, reset timer, don't flip */
		if (ma.duty_cycle == 100 && state == 1) 
			ma.t->tv_nsec += ma.period;
		/* if duty_cycle == 0, reset timer, don't flip */
		else if (ma.duty_cycle == 0 && state == 0)
			ma.t->tv_nsec += ma.period;
		else {
			out(ma.n_pin, &state);
			/* if new state is high, stay high for duty_cycle/100 * period,
			 * if new state is low, stay low for period - (duty_cycle/100 * period)
			 */
			if (state)
				ma.t->tv_nsec += ma.duty_cycle * ma.period / 100;
			else 
				ma.t->tv_nsec += ma.period - ma.duty_cycle * ma.period / 100;
		}
		tsnorm(ma.t);
	}
}


void setup()
{
  wiringPiSetup();
  
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  pinMode(motor_dir, OUTPUT) ;
  pinMode(motor_pwm, OUTPUT);
  
  pinMode(encoder1PinA, INPUT); 
  pinMode(encoder1PinB, INPUT); 
  pinMode(motor_dir1, OUTPUT) ;
  pinMode(motor_pwm1, PWM_OUTPUT);
  
  Send_setup();
  
}

 

int main()
{
  
  setup();
  
  ma.duty_cycle = 0;  
  wiringPiISR(encoder0PinA, INT_EDGE_BOTH, doEncoderA);
  wiringPiISR(encoder1PinA, INT_EDGE_BOTH, doEncoderB);

    pthread_t t;
    pthread_t u;
    pthread_create(&t, 0, PWM_thread,  (void*)0);
    pthread_create(&u, 0, Receive_UDP_thread,  (void*)1); 
    double period=10000000;
    struct timespec t_pwm,temp;
    clock_gettime(0, &t_pwm);

    double times[200];
    double c = 0;

	

    while (1) //(ctr<200) 
    {
        clock_nanosleep(0, TIMER_ABSTIME, &t_pwm, NULL);
	clock_gettime(0,&t_pwm);
	double a = t_pwm.tv_nsec;
	PID_calc();
        t_pwm.tv_nsec =a +  period;// - b + a;
        tsnorm(&t_pwm);
    	//c=a;
   }
  return 0;
  
}
 
void doEncoderA(void){
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
 }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH ) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //delayMicroseconds(10);
}

void doEncoderB(void){
	if (digitalRead(encoder1PinA) == HIGH) {
		if (digitalRead(encoder1PinB) == LOW) {
			encoder1Pos = encoder1Pos + 1;	// CW
		} else {
			encoder1Pos = encoder1Pos - 1;	// CCW
		}
	} else {
		if (digitalRead(encoder1PinB) == HIGH) {
			encoder1Pos = encoder1Pos + 1;	// CW
		} else {
			encoder1Pos = encoder1Pos - 1;	// CCW
			}
		}
} 

float mov_avg_diff_count[2];
int avg_ctr = 0; 
int max_avg_ctr = 50;     
int  mov_avg_buffer1[50];
int  mov_avg_buffer2[50];
int mov_avg_buffer_sum[2] = {0, 0};
int to_be_eliminated[2];
int temp = 0;
bool full = false;
char send_str[7];
float cur_rpm[3];
float count = 0.0;
int diff_encoder_count[2];
int last_encoder_count[2];


void PID_calc()
{
         // printf("\n En:   %d", encoder0Pos);
         diff_encoder_count[LEFT] = encoder0Pos - last_encoder_count[LEFT];
         diff_encoder_count[RIGHT] = encoder1Pos - last_encoder_count[RIGHT];
         
         last_encoder_count[LEFT] = encoder0Pos;
         last_encoder_count[RIGHT] = encoder1Pos;
              
         to_be_eliminated[LEFT] = mov_avg_buffer1[temp];
         to_be_eliminated[RIGHT] = mov_avg_buffer2[temp];
         
         mov_avg_buffer1[temp] = diff_encoder_count[LEFT];
         mov_avg_buffer2[temp] = diff_encoder_count[RIGHT];
         if (temp<49) temp++;
         else { temp = 0; full = true;}
         
         mov_avg_buffer_sum[LEFT] = mov_avg_buffer_sum[LEFT] + diff_encoder_count[LEFT] - to_be_eliminated[LEFT];
         mov_avg_buffer_sum[RIGHT] = mov_avg_buffer_sum[RIGHT] + diff_encoder_count[RIGHT] - to_be_eliminated[RIGHT];
         
         if(full==true) {
			 mov_avg_diff_count[LEFT] = ((float)mov_avg_buffer_sum[LEFT])/50.0;
			 mov_avg_diff_count[RIGHT] = ((float)mov_avg_buffer_sum[RIGHT])/50.0;
		 } else {
			 mov_avg_diff_count[LEFT] = ((float)mov_avg_buffer_sum[LEFT])/((float)temp);
			 mov_avg_diff_count[RIGHT] = ((float)mov_avg_buffer_sum[RIGHT])/((float)temp);
		 }
        
        cur_rpm[0] = (mov_avg_diff_count[LEFT]*resolution_encoder)*60*sampling_freq/360;
        cur_rpm[1] = (mov_avg_diff_count[RIGHT]*resolution_encoder)*60*sampling_freq/360;
        cur_rpm[2] = count;

        if((int)count%250==0) printf("\nLeft:  %f     Right:  %f", cur_rpm[0], cur_rpm[1]); 

        Send_UDP(cur_rpm);
        
        if (count > 10000)
        count = 0;

        count++;
}
