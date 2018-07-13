/*******************************************************************************
* rc_project_template.c
*
* This is meant to be a skeleton program for robotics cape projects.
* Change this description and file name
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h>
// main roboticscape API header
#include <roboticscape.h>
#include <string.h>
#include <pthread.h>

#define timestep 0.01
#define NUMTHREADS 1
// function declarations
void on_pause_pressed();
void on_pause_released();

pthread_t thread;

float pos;
float e2[2];
float u2[2];

void* thread1(void* ptr)
{
	while(rc_get_state()!=EXITING){
		printf("Thread 2 at work\n");
		pos=rc_get_encoder_pos(4);
		printf("Position is: %f",pos);
		e2[1]=pos;
		u2[1]=-0.00021*u2[0]+0.00012*e2[1]-0;//0.0012*e2[0];
		//rc_set_motor(3,-u2[1]);
		//rc_set_motor(2,u2[1]); 
		
		printf("motor out: %f\n", u2[1]);
		e2[0]=e2[1];
		usleep(10000);
	}
	return NULL;
}
/*******************************************************************************
* int main()
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

	// do your own initialization here
	printf("\nIMU TEST\n");

	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);
	rc_enable_motors();
	//int rc_set_imu_config_to_defaults(rc_imu_config_t *conf);
	rc_imu_data_t imu_data;
	rc_imu_config_t imu_config = rc_default_imu_config();
	rc_initialize_imu_dmp(&imu_data, imu_config);
	if(rc_initialize_imu_dmp(&imu_data,imu_config)){
		fprintf(stderr,"rc_initialize_imu_dmp failed\n");
		return -1;
	}
	float theta_g[2]; float theta_a[2];
	theta_g[0]=0; // initialize gyro matrix
	//float theta_a_raw=atan2(-imu_data.accel[2],imu_data.accel[1]);
	float theta_a_filtered[2];
	float theta_g_filtered[2];
	float theta_f;
	float u[2]; u[0]=1;
	float e[2]; e[0]=0;
	e2[0]=0;u2[0]=0;

	theta_g_filtered[0]=imu_data.gyro[0]; // initialize theta g filtered
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING);

	pthread_create(&thread,NULL,thread1,NULL);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){

			printf("IMU\n");
			float theta_a_raw=atan2(-imu_data.accel[2],imu_data.accel[1]);
			theta_a[1]=theta_a_raw;
			theta_g[1]=theta_g[0]+timestep*imu_data.gyro[0]*(3.14159/180);
			//float theta_g_raw=theta_g[1]*(3.14159/180); //converts to radians

			//printf("X: %.3f |Y: %.3f |Z: %.3f |theta_a_raw: %.3f| theta_g_raw: %.3f\n",imu_data.accel[0],imu_data.accel[1],imu_data.accel[2],theta_a_raw,theta_g_raw);
			printf("theta_a_raw: %.3f| theta_g_raw: %.3f\n",theta_a_raw,theta_g[1]);



			//CREATING DIGITAL FILTER
			float wc=0.5; // specify crossover freq
			// Low Pass Filter
			theta_a_filtered[1]=(1/(200+wc))*((200-wc)*theta_a_filtered[0]+wc*theta_a[1]+wc*theta_a[0]);
			theta_a_filtered[0]=theta_a_filtered[1];
			// Low Pass Filter
			theta_g_filtered[1]=((200-wc)/(200+wc))*theta_g_filtered[0]+(200/(200+wc))*(theta_g[1]-theta_g[0]);
			theta_g_filtered[0]=theta_g_filtered[1];
			// Complementary Filter
			theta_f=theta_a_filtered[1]+theta_g_filtered[1]+0.35;

			printf("Theta Filtered: %.3f\n",theta_f);
			theta_a[0]=theta_a[1];
			theta_g[0]=theta_g[1];
			if(theta_f<0.9 && theta_f>-0.8){
			// create error term
			e[1]=theta_f;
			//create feedback with difference equation
			u[1]=u[0]-1.07*e[1]+1*e[0];
			printf("Motor Duty Output: %.3f\n",u[1]);
			//create motor output

			printf("%.3f\n",theta_f);
			rc_set_motor(1,u[1]+u2[1]);
			rc_set_motor(4,-u[1]-u2[1]);

			e[0]=e[1];
			u[0]=u[1];
			//rc_set_motor_free_spin_all();
			} else {
				rc_set_motor_free_spin_all();
				break;
			}
		}
		else if(rc_get_state()==PAUSED){
			printf("paused\n");
			rc_set_motor_free_spin_all();
		}
		// always sleep at some point
		usleep(10000);
	}

	// exit cleanly
	rc_power_off_imu();
	rc_cleanup();
	rc_disable_motors();
	return 0;
}


/*******************************************************************************
* void on_pause_released()
*
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed()
*
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

// This point onwards my own code
// High Pass
/*
float highpassfilter(float angle1, float angle2) {
	theta_g_filtered= ((a-wc)/(a+wc))*
	return theta_g_filtered;
}*/