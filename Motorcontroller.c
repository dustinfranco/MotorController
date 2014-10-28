//Lab 4 ECE 4760
//some code taken from Lab 2 (LCD), 3 and 4 (TRT and Computer input examples)


//Library Inclusions

//TRT inclusions
#include "trtSettings.h"
#include "trtkernel_1284.c"
#include "trtUart.h"
#include "trtUart.c"
//General use inclusions for basic operations
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h> // needed for lcd_lib
#include <math.h>
#include <inttypes.h>
//LCD inclusions
#include "lcd_lib.h"
#include "lcd_lib.c"

//Part of Uart Initialization
FILE uart_str=FDEV_SETUP_STREAM(uart_putchar,uart_getchar,_FDEV_SETUP_RW);

//Semaphore Declarations

#define SEM_RX_ISR_SIGNAL 1 //unused
#define SEM_SHARED 3 //Only used semaphore
#define SEM_TX_WAIT 4 //unused


//Variable declarations for the PID/motor control
volatile int motor_period_ovf;
volatile int motor_period;
volatile int prop_Gain;
volatile int diff_Gain;
volatile int integral_Gain;
volatile int desired_rpm;
volatile int motor_period_two;

//some LCD related constants taken from Lab 1
const int8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const int8_t LCD_line[] PROGMEM = "line 1\0";
const int8_t LCD_number[] PROGMEM = "Number=\0";
int8_t lcd_buffer[17];    // LCD display buffer
uint16_t count;            // a number to display on the LCD  
uint8_t anipos, dir;    // move a character around 

//void arguments passed to tasks
int args[2];

// LCD initialization function
void init_lcd(void) 
{
	//fprintf(stdout, "initializing lcd");
    LCDinit();    //initialize the display
    LCDcursorOFF();		//turn off the blinking cursor
    LCDclr();                //clear the display
    LCDGotoXY(0,0);		//go to the top left coordinate of the LCD screen
}


//BEGINNING OF ISRS

//PIN D2 interrupt vector, runs when the fan blade interrupts the IR
ISR (INT0_vect)
{
//calculate the total number of cycles between the last interrupt and this interrupt
    motor_period = TCNT2 + motor_period_ovf; 
	//reset the count and overflow variables to begin counting towards the next interrupt
    TCNT2 = 0;
    motor_period_ovf = 0;
}

//Timer 2 overflow vector, this increases the allowable time between interrupts
ISR (TIMER2_OVF_vect)
{
//if there's an overflow, add 256 (at 256 there's an overflow) to this variable, which
//is later added to TCNT2 to form our total motor period
    motor_period_ovf = motor_period_ovf + 256;
}

//END ISRS

// Task 1: Control Loop
void control(void* args)
{                                    //BEGINNING OF CONTROL
//TRT release and dead variables for control loop
    uint32_t contrel, contdead ;
//variable declarations, mostly floats due to many type errors in our equations
	double rotation_time;
	float measured_rpm;
	float error;
	float prev_error = 0;
	float derivative;
	float integral = 0;
	float ds;
	float Imax=1;
	float Control_out;
	float Control_out_scaled;
	float measured_rpm_scaled;
	//n is our conversion from RPM to cycles for the PWM outputs
	float n = 0.085;
	float m;
    int insign;
	//enter hte infinite loop for this task
	while(1)
    {
	ds = (float)diff_Gain/.02; // this is used because we have a constant time between control tasks.  
	//wait on the semaphore and retrieve rotation time, to ensure that rotation time isn't changed while
	//retrieving it
	trtWait(SEM_SHARED);
	//Taking the motor period value from our ISRs, we scale it accordingly from cycles to seconds
	rotation_time = (float)(motor_period * 448.0 * pow(10,-6));
	//signal that we are done taking rotation time
	trtSignal(SEM_SHARED);
	//measured RPM, since there are 60 seconds in a minute, and rotation time is a period, not a frequency
	//we calculate this by (60/Period of rotation (in seconds)) to get the rotations/minute
	measured_rpm = (float) (60.0/rotation_time);
	//fprintf(stdout, "m_rpm = %f", measured_rpm);
	//fprintf(stdout, "d_rpm = %f", desired_rpm);
	//calculate the difference between our desired RPM and our estimated RPM
	error = desired_rpm - measured_rpm;
	//Make sure the error is always greater than or equal to 0
	if(error<0){
	error=0;
	}
	//calculate the derivative
	derivative = error - prev_error;	
	//calculate the integral
	integral = integral + error;
	//This is used to check that the integral doesn't change  signs instantly without being 0 first
	if(insign==0&&integral<0)
	{
	integral=0;
	}
	if(insign==1&&integral>0)
	{
	integral=0;
	}
	//Our control equation given by Bruce in class
	Control_out = (float)prop_Gain*error + ds*(error-prev_error) + (float)integral_Gain*fminf(Imax, integral);
	//Scale our control to a value between 0 and 255 to be used for OCR0A
	Control_out_scaled = Control_out*n;
	//Scale the measured RPM for OCR0B
	measured_rpm_scaled = measured_rpm*n;
	//Scale the desired RPM
	m = n* (float)desired_rpm;
	//0 is our maximum value, 255 is our minimum value
	//if we exceed our maximum value (IE, control_out_scaled>255) then set OCR0A to 0
	if(255-(int)Control_out_scaled<0)
	{
	OCR0A=0;
	}
	else
	{
	//calculate it normally if that is not the case
	OCR0A =  255-(int)Control_out_scaled;
	}
	//OCR0B declaration as a scaled value of the measured RPM
	OCR0B=255-(int)(measured_rpm_scaled);
	
	//Debugging print statements
	
	//fprintf(stdout, "%i\n\r", motor_period);
	//	fprintf(stdout, "%i\n\r", motor_period_two);
	//fprintf(stdout, "%f\n\r", rotation_time);
	//fprintf(stdout, "%f\n\r", measured_rpm);
	//fprintf(stdout, "%i", OCR0B);
	//OCR0A = 0;
	//OCR0A = 255;
	//Keep track of the previous error
	error = prev_error;
	//Take note of the current sign of the integral
	if(integral>0)
	{
	insign=1;
	}
	else
	{insign=0;}
    
    //OCR0A is B.3, OCR0B is B.4
    //OCR0A=rpm*Conversion Factor based on scaling, processor speed, and divided by 255
    //OCR0B=rpm * conversion factor based on RPM, scaled so that 255= 3000 rpm, 0 = 0 rpm
    
    //Set release and dead times (.02 runs 50 times a second ideally)
    contrel = trtCurrentTime() + SECONDS2TICKS(0.02);
    // set dead time to slightly larger than the release time so it acts at a constant rate (so we have dt).  
    contdead = trtCurrentTime() + SECONDS2TICKS(0.04);
    // sleep given these parameters
    trtSleepUntil(contrel, contdead);

    }
}                                    //END OF CONTROL LOOP

// Task 2: Read From Computer
void serialComm(void* args)
  {                                 //BEGINNING OF SERIALCOMM
  //Variable declaration for this task
    uint16_t paramVal ;
    char cmd[6] ;
	//enter the infinite loop for this task
    while(1)
    {
        // commands:
        // 's 300' sets the desired motor speed to 300
        // 'p 10'  sets the proportional gain to 10
        // 'i 2'   sets the integral gain to 2
        // 'd 5.2' sets the differential gain to 5.2
		//print this to indicate it wants another command
        fprintf(stdout, ">") ;
		//wait for the input from the computer and interpret it into which variable to change
		//and what to change it to
        fscanf(stdin, "%s%d", cmd, &paramVal) ;
        

        // update shared variables used in the PID equation
        trtWait(SEM_SHARED) ;    
		//the IF statements determine which value to change, and within them the value
		//is changed to whatever inputted value there is
        if (cmd[0] == 's')
         {
		    desired_rpm = paramVal ;
            //fprintf(stdout, "you changed desiredMotorSpeed to %i", paramVal); 
        }
		if (cmd[0] == 'p')        
         {
		    prop_Gain = paramVal ;
            //fprintf(stdout, "you changed propGain to %i", paramVal); 
        }
		if (cmd[0] == 'i')        
         {
		    integral_Gain  = paramVal ;
            //fprintf(stdout, "you changed integralGain to %i", paramVal); 
        }
		if (cmd[0] == 'd')
		{
            diff_Gain = paramVal;
            //fprintf(stdout, "you changed diffGain to %i", paramVal); 
        }
		//signal that we are done handling shared variables
		trtSignal(SEM_SHARED);
        
    }
  }                                    //END OF SERIAL COMM

  //Task 3: Update the LCD
void screenupdate(void* args)
{                                    //BEGINNING OF LCD UPDATE

	//LCD update release and dead times for TRT
    uint32_t screenrel, screendead ;
	//the character array for the message
	char lmes[6];
	//motor variables
	float rotation_time;
	float measured_rpm;
	//start the task by clearing the LCD entirely.
	LCDclr();
    while(1)
    {
	//fprintf(stdout,"LCD");
	                //clear the display
	//wait for the shared variable semaphore, because we handle motor period here
	trtWait(SEM_SHARED);
	
	//fprintf(stdout, "MP = %d\n\r", motor_period) ;
	//convert the rotation time the same way as we did in the control loop
	rotation_time = (float)(motor_period * 448.0 * pow(10,-6));
	//signal that we are done with the motor period variable
	trtSignal(SEM_SHARED);
	
	//measured RPS to measured RPM
	measured_rpm = (float) (60.0/rotation_time);
	//make sure we are at the top left of the screen
    LCDGotoXY(0,0);
	//print the desired rpm to the top line
	sprintf(lmes, "Desired:  %i   ", (int)desired_rpm);
    LCDstring(lmes, strlen(lmes));	
	//go to the first character of the second line
    LCDGotoXY(0,1);
	//print the measured rpm to the second line
	sprintf(lmes, "Measured: %i   ", (int)measured_rpm);
    LCDstring(lmes, strlen(lmes));
	//fprintf(stdout, "Entering LCD Task") ;
       
	//set the release time to .1 seconds
    screenrel = trtCurrentTime() + SECONDS2TICKS(0.1);
    //set dead time to .2 seconds from now
    screendead = trtCurrentTime() + SECONDS2TICKS(0.2);
    //sleep given these parameters, which should run the LCD update between 5-10 times a second
    trtSleepUntil(screenrel, screendead);
    }
}                                    //END OF LCD update

  int main()
  {    
  //initialize PID values and RPM
 desired_rpm=3000;
 prop_Gain=1;
 diff_Gain=1;
 integral_Gain=1;

 
	//set stdout for Uart
   stdout=stdin=stderr=&uart_str;
 
 
  //TRT STUFF
  //fprintf(stdout,"\n\r%d\n\r",PINB);
    

  // start TRT Kernel
  trtInitKernel(80); // 80 bytes for the idle task stack
  //initialize the TRT uart
  trt_uart_init();
 fprintf(stdout,"\n\r...TRT BEGIN ...\n\r");
  // Semaphores for Uart
  trtCreateSemaphore(SEM_RX_ISR_SIGNAL, 0) ; // uart receive ISR semaphore
  trtCreateSemaphore(SEM_STRING_DONE,0) ;  // user typed <enter>
  
  //Semaphores Used By Us
  trtCreateSemaphore(SEM_TX_WAIT, 1) ; // This is actually not used, but leftover from a previous revision
  trtCreateSemaphore(SEM_SHARED, 1) ; // protect shared variable

 // Creating tasks, we gave them ample stack size and execute time to ensure that they would function properly
  trtCreateTask(control, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.1), &(args[1]));
  trtCreateTask(serialComm, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.5), &(args[1]));
  trtCreateTask(screenupdate, 200, SECONDS2TICKS(0.1), SECONDS2TICKS(0.5), &(args[0]));
 


	//initialize the LCD
    init_lcd();                          
    //IN MAIN:
    EIMSK = 1<<INT0 ; // turn on int0 on pin D2
    EICRA = 3 ;      // rising edge
    //Turn on timer 2 to be read in int0 ISR
    TCCR2B = 7 ; // divide by 1024
    //Turn on timer 2 overflow ISR for double precision time
    TIMSK2 = 1 ;
    

//PWM STUFF
	//turn on OCR0A and OCR0B
   TCCR0A = (1<<COM0A0) | (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) | (1<<COM0B0) | (1<<COM0B1);
   //make pin B3 and B4 outputs
   DDRB = (1<<PINB3|1<<PINB4) ;
   //make pind D2 and input for interrupt 0
   DDRD = (0<<PIND2);
   // timer 0 runs at full rate
   TCCR0B = 1 ; 


    sei();//NEVER FORGET
    while(1)
    {
    }
  }                                //END EVERYTHING
