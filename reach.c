/*
 * File:   reach.c
 * Author: shaarawy
 *
 * Created on November 26, 2022, 2:21 PM
 */

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 16000000
#include <xc.h>

//#define trigger RB1
//#define echo    RB2

// global variables decleration
int up, left, right; // three variables to hold the position(up how much to go forware)
//int right_encoder, left_encoder; // to hold how many holes passed through the encoder 
int duty_cycle; 
int forward_dist;
int time_taken;
int start = 0; // when zero thus no location is defined so car no move, when 1 then we start the main code
int reach = 0; // when 1 then the car has reached the location and it must stop, initially it is zero
int test_pwm = 0;

int up_counts = 0; // var to store how many holes of the shaft were counted at each iteration of the main loop
int up_flag = 0;
int up_dist = 0;

int back_counts = 0; // var to store how many holes of the shaft were counted while going back off
int back_flag = 0; // var to tell whether we are moving back or not 
int back_dist = 0; // var to store how far we went back 

int turn_right_counts = 0; // var to hold how many holes of the shaft were counted while turning (right or left)
int turn_right_flag = 0; // flag to tell whether we are truning or not

int turn_left_counts = 0;
int turn_left_flag = 0;

// functions decleration  
void PWM_Init(void);
void PWM_right_init(void);
void set_DC(int x);
void set_DC_right(int x);
void avoid(void);
void back_off(void);
void forward(void);
void stop(void);
void recenter_left(void);
void recenter_right(void);
void turn_left(void);
void turn_right(void);
//void calculate_distance(void);
void reverse(void);
void tmr1_init(void);
int cal_dist(void);

//********************************************************************************
//--------------------------------
// Functions Declarations
void UART_RX_Init(void);
// Globals
uint8_t UART_Buffer = 0;
//--------------------------------

// Functions Definitions
 
void UART_RX_Init()
{
  BRGH = 1; // Set For High-Speed Baud Rate
  SPBRG = 103; // Set The Baud Rate To Be 9600 bps
  // Enable The Ascynchronous Serial Port
  SYNC = 0;
  SPEN = 1;
  // Set The RX-TX Pins to be in UART mode (not io)
  TRISC6 = 1; // As stated in the datasheet
  TRISC7 = 1; // As stated in the datasheet
  //--[ Enable UART Receiving Interrupts ]--
  RCIE = 1; // UART Receving Interrupt Enable Bit
  PEIE = 1; // Peripherals Interrupt Enable Bit
  GIE = 1; // Global Interrupt Enable Bit
  //------------------
  CREN = 1; // Enable Data Continous Reception
}
 //****************************************************************************************

// function initialization 

// PMW function
void PWM_Init(void)
{
    //////////////// the function is tested and it is working //////////////////////
    // for this config the max DC is 200 which gives full power
  //--[ Configure The CCP Module For PWM Mode ]--
  CCP1M3 = 1;
  CCP1M2 = 1;
  TRISC2 = 0; // The CCP1 Output Pin (PWM)
  // Set The PWM Frequency (10kHz)
  PR2 = 99;
  // Set The PS For Timer2 (1:4 Ratio)
  T2CKPS0 = 1;
  T2CKPS1 = 0;
  // Start CCP1 PWM !
  TMR2ON = 1;
}

// function to generate the pwm signal for the right motor (slow one)
void PWM_right_init(void)
{
    //////////////// the function is tested and it is working //////////////////////
   // for this config the max DC is 400 which gives full power
  //--[ Configure The CCP Module For PWM Mode ]--
  CCP2M3 = 1;
  CCP2M2 = 1;
  TRISC1 = 0; // The CCP1 Output Pin (PWM)
  // Set The PWM Frequency (10kHz)
  PR2 = 99;
  // Set The PS For Timer2 (1:4 Ratio)
  T2CKPS0 = 1;
  T2CKPS1 = 0;
  // Start CCP1 PWM !
  TMR2ON = 1;
}

// function to set the duty cycle of the pwm 
void set_DC(int x){
   // set the ducty cycle 
  CCP1Y = x & 1;
  CCP1X = x & 2;
  CCPR1L = x >> 2;
}

// function to set the duty cycle of the pwm right motor  
void set_DC_right(int x){
   // set the ducty cycle 
  CCP2Y = x & 1;
  CCP2X = x & 2;
  CCPR2L = x >> 2;
}

// tmr_init function to initialize timer 1 to be used with cal dis method 
void tmr1_init(){
    // Clear The Pre-Scaler Select Bits
  T1CKPS0=0;
  T1CKPS1=0;
  // Choose The Local Clock As Clock Source
  TMR1CS=0;
    //T1CON = 0x10;
}

// function to calculate the distance 
int cal_dist(void){
  int distance=0;
  //TMR1ON = 0;
  TMR1=0;
  // Send Trigger Pulse To The Sensor
  RD5 = 1;
  __delay_us(10);
  RD5 = 0;
  // Wait For The Echo Pulse From The Sensor
  while(!RD4);
  // Turn ON Timer Module
  TMR1ON = 1;
  // Wait Until The Pulse Ends
  while(RD4);
  // Turn OFF The Timer
  TMR1ON=0;
  // Calculate The Distance Using The Equation
  distance = (TMR1L | (TMR1H<<8));
  distance = distance*(0.00425);
  return distance;
}

// forward function 
void forward(void){
    RC4=1; RC5=0; //Motor 1 forward (right motor)
    RD6=1; RD7=0; //Motor 2 forward (left motor)
}

//reverse function
void reverse(void){
    RC4=0; RC5=1; //Motor 1 reverse (right motor)
    RD6=0; RD7=1; //Motor 2 reverse (left motor)
}

// back_off function initialiation 
void back_off(void){
    reverse();
    
    back_dist = 0;
    back_flag = 1;
    
    while(back_dist < 5){
        __delay_ms(50);
        back_dist = (back_counts/20)*19;
    }
    up = up + back_dist; // to add this to the path needed to be done
    
    back_flag = 0;
    back_dist = 0;
    back_counts = 0;
    
    stop(); // stop the car
    __delay_ms(1000); // stop the car, just standby period
}

// stop function to stop the car
void stop(void){
    RC4 = 0; RC5 = 0; // Motor 1 stop (right motor)
    RD6=0; RD7=0; //Motor 2 stop (left motor)
}

//turn left function 
void turn_left(void){
    
    RC4=1; RC5=0; //Motor 1 forward (right motor)
    RD6=0; RD7=0; //Motor 2 stop (left motor)
    
    turn_left_counts = 0;
    turn_left_flag = 1;
    
    while(turn_left_counts < 21); 
    
    turn_left_flag = 0;
    turn_left_counts = 0;
  
    stop(); // stop the car, standby period
    __delay_ms(1000);
}

// turn right function 
void turn_right(void){
    
    RC4=0; RC5=0; //Motor 1 stop (right motor)
    RD6=1; RD7=0; //Motor 2 forward (left motor)
   
    turn_right_counts = 0;
    turn_right_flag = 1;
    
    while(turn_right_counts < 21);
    
    turn_right_counts = 0;
    turn_right_flag = 0;
    
    stop(); // stop the car, just standby period
    __delay_ms(1000);
}

// recentre left function, to recenter after turning left
void recenter_left(void){
    
    up_counts = 0;
    up_flag = 1;
    up_dist = 0;
    
    while(up_dist < 10){
        forward();
        up_dist = (up_counts/20)*19;
    }
    
    right = right + up_dist;
    
    up_counts = 0;
    up_flag = 1;
    up_dist = 0;
    
    while(RD3 == 0){
        forward();
        __delay_ms(50);
    }
    
     right = right + ((up_counts/20)*19);
    
    stop(); // stop the car for standby period then recentre the car
    
    up_counts = 0;
    up_flag = 0;
    up_dist = 0;
    
    __delay_ms(1000);
    
    turn_right(); // turn the car right so now it has the same reference 
    
}

// recentre right function, to recentere the car after turning right
void recenter_right(void){
    
    up_counts = 0;
    up_flag = 1;
    up_dist = 0;
    
    while(up_dist < 10){
        forward();
        up_dist = (up_counts/20)*19;
    }
    
    left = left + up_dist;
    
    up_counts = 0;
    up_flag = 1;
    up_dist = 0;
    
    while(RD2 == 0){
        forward();
        __delay_ms(50);
    }
    
    left = left + ((up_counts/20)*19);
    
    stop(); // standby period
    
    up_counts = 0;
    up_flag = 0;
    
    __delay_ms(1000);
    
    turn_left(); // turn the car left so now it has the same reference as before
}

void avoid(void){
    // if left sensor is blocked, then turn right
    if (RD2 == 0 && RD3 == 1){
        back_off();
        turn_right();
        recenter_right();
    }
    
    // if right sensor is blocked then turn left
    else if(RD2 == 1 && RD3 == 0){
        back_off();
        turn_left();
        recenter_left();
    }
    
    // if both sensors are open then to turn right, although turn left will also do the job
    else if(RD2 == 1 && RD3 == 1){
        back_off();
        turn_right();
        recenter_right();
    }
    
    //if both sensors are blocked, then to move backward till a path open
    else if(RD2 == 0 && RD3 == 0){
        back_counts = 0;
        back_flag = 1;
        // to go back till find an open path
        while(RD2 == 0 && RD3 == 0){
            reverse();
        }
        
        stop();
        __delay_ms(1000);
        
        up = up + ((back_counts/20)*19);
        back_counts = 0;
        back_flag = 0;
        
        // it go out of the loop when a path is open, so to check which path is open
        // if the right path is open, then turn right and recenter right
        if(RD3 == 1){
            turn_right();
            recenter_right();
        }
        // if the left path is open, then turn left and recenter left
        else {
            turn_left();
            recenter_left();
        }
    }
    
}

void main(void) {
    
    // enable port B change interrupt to track the distance using optical encoder
    GIE = 1; // enable general interrupt 
    
    RBIE = 1; // enable port B change interrupt
    RBIF = 0; // to reset the flag of the interrupt
    //TRISB4 = 1; // set RB4 as input pin, to be connected to the right wheel encoder
    TRISB5 = 1; // set RB5 as input pin, to be connected to the left wheel encoder
    
    INTE = 1;
    INTF = 0; //clear interrupt flag bit
    INTEDG = 1;
    TRISB0 = 1;
    
    // ultra sonic pins configuration 
    TRISD5 = 0; //Trigger pin of US sensor is sent as output pin
    RD5 = 0; // initially zero
    TRISD4 = 1; //Echo pin of US sensor is set as input pin
    RD4 = 0;
    
    
   // IR sensors pins configuration 
    TRISD2 = 1; // left sensor, defined as input pin, when low there is obstacle
    TRISD3 = 1; // right sensor, defined as input pin, when low there is obstacle
    RD2 = 0;
    RD3 = 0;
    
    
    // pins configuration for the motors 
    TRISC4 = 0; TRISC5 = 0; //Motor 1 (right motor) pins declared as output, Rc4=> in1 Rc5=> in2
    TRISD6 = 0; TRISD7 = 0; //Motor 2 (left motor) pins declared as output, Rc6 => in3 Rc7 =>in4
    RC4 = 0;
    RC5 = 0;
    RD6 = 0;
    RD7 = 0;
    //PORTC =0;
    // pin for the forward IR sensor 
    //TRISD4 = 1; //q forward IR sensor, if low then object in fron of the car
    
    TRISD1 = 1; // front IR sensor 
    RD1 = 0;
    TRISD0 = 1; // another front IR sensor 
    RD0 = 0;
  
    tmr1_init();
    PWM_Init();
    set_DC(260);
    PWM_right_init();
    set_DC_right(260);
    
    
    UART_RX_Init(); // Initialize The UART in Master Mode @ 9600bps
    
    while(1){
        while(start == 1){
            up_counts = 0;
            up_flag = 1;
            forward();
            __delay_ms(50);
            up = up - up_counts;
            up_counts = 0;
            up_flag = 0;
            //forward_dist = cal_dist();
            if(RD1 == 0 | RD0 == 0){
                //up_flag = 0;
                stop();
                __delay_ms(1000);
                avoid();
            }
            if((up < 5) & (left < 5) & (right < 5)){
                start = 0;
                stop();
            }
            else if(up < 5){
                    if(left > right){
                        up = left - right;
                        stop();
                        __delay_ms(1000);
                        //up_flag = 0;
                        turn_left();
                    }
                    if(right > left){
                        up = right - left;
                        stop();
                        __delay_ms(1000);
                        //up_flag = 0;
                        turn_right();
                    }
                    right = 0;
                    left = 0;
            }
            
        } 
        
    }
    return;
}

void __interrupt() ISR(void){
    
    if(INTF == 1){
        //right_encoder = right_encoder + 1; // increment the right var, holding how many holes passed through encoder
        if(up_flag == 1)
            up_counts = up_counts + 1;
        else if(back_flag == 1)
            back_counts = back_counts + 1;
        else if(turn_left_flag == 1)
            turn_left_counts = turn_left_counts + 1;
        INTF=0;
    }
    
    // to check that it is port B change interrupt
    if(RBIF == 1){
        // to check if it is the left wheel 
        if(RB5 == 0){
            //left_encoder = left_encoder + 1; // increment the left var, holding how many holes passed through encoder
            if(turn_right_flag == 1)
                turn_right_counts = turn_right_counts + 1;
        }
        RBIF = 0; // reset the interrupt flag
    }
    
    //*******************************************************************
    if (RCIF == 1)
  {
    UART_Buffer = RCREG; // Read The Received Data Buffer
 
    // This could have been done within the main loop. Since it's not
    // Excessive processing, so it's OK to do it here below
    if(UART_Buffer == 49){
        up = 840;
        left = 0;
        right = 160;
        //forward();
    }
    if(UART_Buffer == 50){
        up = 160;
        left = 0;
        right = 840;
        //reverse();
    }
    start = 1;
    RCIF = 0; // Clear The Interrupt Flag
  }
    //******************************************************************************
}