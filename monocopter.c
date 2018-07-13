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
#include <rc_usefulincludes.h>
// main roboticscape API header
#include <roboticscape.h>
#include <string.h>
#include <pthread.h>
#include <termios.h>
#include <time.h>

#define HIGH 1
#define LOW 0

static struct termios old, new;
clock_t begin;

char move_c;
float move_n;
float power=0;
int time_sys=0;
float accel_x,accel_y,accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
int gpio_val, gpio_val2, gpio_echo;

// function declarations
void on_pause_pressed();
void on_pause_released();

pthread_t thread;

//Initialize automatic keyboard input
void initTermios(int echo);
void resetTermios(void);
char getch(void);
void resetTermios(void);

//this thread handles the controls
void* thread1(void* ptr){
	while(rc_get_state()!=EXITING){
		move_c = getch();
			if (move_c=='w'){
				move_n=0.025;
			}
			else if (move_c=='s'){
				move_n=-0.025;
			}
			if (power<0){power=power+0.025;}
			else if (power>0.8){power=power-0.025;}
			power=power+move_n;
			//printf("%f\n",power);
			usleep(100000);
	}
	return NULL;
}

//this thread handles the display of information
void* thread2(void* ptr){
	while(rc_get_state()!=EXITING){
		//Observer mode
		printf("Time:%ds|Power:%f\n",time_sys,power);
		printf("Accel(XYZ):%f %f %f\n",accel_x,accel_y,accel_z);

		//developer mode
		//printf("%d,%.1f,%.2f,%.2f,%.2f,",time_sys,power,accel_x,accel_y,accel_z);
		//printf("%4.3f,%4.3f,%4.3f\n",gyro_x,gyro_y,gyro_z);
		//printf("%f,%f,%f\n",mag_x,mag_y,mag_z);
		//time_sys++
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

	//rc_send_esc_pulse_normalized(1, 0);
	// do your own initialization here
	//printf("\nMello XeagleBone\n");
	printf("Testing Monocopter\n");
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// initialize IMUs
	int rc_set_imu_config_to_defaults(rc_imu_config_t *conf);
	rc_imu_data_t imu_data;
	rc_imu_config_t imu_config = rc_default_imu_config();
	rc_initialize_imu_dmp(&imu_data, imu_config);
	if(rc_initialize_imu_dmp(&imu_data,imu_config)){
		fprintf(stderr,"rc_initialize_imu_dmp failed\n");
		return -1;
	}

	rc_enable_servo_power_rail();
	rc_gpio_export(97);
	// initialize GPIOs
	//GP1	| GND/3.3V/GPIO1_4(PIN98)/GPIO1_3(PIN97)|
	
	// rc_gpio_set_dir(97, OUTPUT_PIN);
	//

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	pthread_create(&thread,NULL,thread1,NULL);
	pthread_create(&thread,NULL,thread2,NULL);
	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			accel_x=imu_data.accel[1];
			accel_y=imu_data.accel[0]; //reorient IMU axis to body axis
			accel_z=imu_data.accel[2];
			gyro_x=imu_data.gyro[1];
			gyro_y=imu_data.gyro[0]; //reorient IMU axis to body axis
			gyro_z=imu_data.gyro[2];
			mag_x=imu_data.mag[1];
			mag_y=imu_data.mag[0]; //reorient IMU axis to body axis
			mag_z=imu_data.mag[2];
			
			
			
			printf("OK");
			//printf("Power is %2f\n",power);
			//rc_send_esc_pulse_normalized(1, power);
			rc_send_servo_pulse_us(1, 1100+1000*power);
		}
		else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
			break;
		}
		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	rc_cleanup(); 
	rc_power_off_imu();
	resetTermios();
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

void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      new.c_lflag |= ECHO; /* set echo mode */
  } else {
      new.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}

/***********Proximity Sensor*************/