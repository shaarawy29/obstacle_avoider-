/*
 * File:   reach.c
 * Author: shaarawy
 *
 * Created on November 26, 2022, 2:21 PM
 */

// CONFIG
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 8000000
#include <xc.h>

// global variables decleration
int up, left, right;
int duty_cycle;
int distance;
int time_taken;
int start = 0; // when zero thus no location is defined so car no move, when 1 then we start the main code
int reach = 0; // when 1 then the car has reached the location and it must stop, initially it is zero
int test_pwm = 0;

// functions decleration  
void PWM_Init();
void set_DC(int x);
void avoid();
void back_off();
void forward();
void stop();
void recenter_left();
void recenter_right();
void turn_left();
void turn_right();
void calculate_distance();
void reverse();


// function initialization 

// PMW function
void PWM_Init()
{
    //////////////// the function is tested and it is working //////////////////////
    // for this config the max DC is 200 which gives full power
  //--[ Configure The CCP Module For PWM Mode ]--
  CCP1M3 = 1;
  CCP1M2 = 1;
  TRISC2 = 0; // The CCP1 Output Pin (PWM)
  // Set The PWM Frequency (10kHz)
  PR2 = 49;
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

// forward function 
void forward(){
    RC4=1; RC5=0; //Motor 1 forward (right motor)
    RC6=1; RC7=0; //Motor 2 forward (left motor)
}

//reverse function
void reverse(){
    RC4=0; RC5=1; //Motor 1 reverse (right motor)
    RC6=0; RC7=1; //Motor 2 reverse (left motor)
}

// back_off function initialiation 
void back_off(){
    RC4=0; RC5=1; //Motor 1 reverse (right motor)
    RC6=0; RC7=1; //Motor 2 reverse (left motor)
    __delay_ms(500);
    up = up + 500; // to add this to the path needed to be done
    
    stop(); // stop the car
    __delay_ms(500); // stop the car, just standby period
}

// stop function to stop the car
void stop(){
    RC4 = 0; RC5 = 0; // Motor 1 stop (right motor)
    RC6=0; RC7=0; //Motor 2 stop (left motor)
}

//turn left function 
void turn_left(){
    RC4=1; RC5=0; //Motor 1 forward (right motor)
    RC6=0; RC7=0; //Motor 2 stop (left motor)
    __delay_ms(500);
  
    stop(); // stop the car, standby period
    __delay_ms(500);
}

// turn right function 
void turn_right(){
    RC4=0; RC5=0; //Motor 1 stop (right motor)
    RC6=1; RC7=0; //Motor 2 forward (left motor)
    __delay_ms(500);
    
    stop(); // stop the car, just standby period
    __delay_ms(500);
}

// recentre left function, to recenter after turning left
void recenter_left(){
    while(RD3 == 0){
        forward();
        __delay_ms(100);
        left = left + 100;
    }
    stop(); // stop the car for standby period then recentre the car
    turn_right(); // turn the car right so now it has the same reference 
    
}

// recentre right function, to recentere the car after turning right
void recenter_right(){
    while(RD2 == 0){
        forward();
        __delay_ms(100);
        right = right + 100;
    }
    stop(); // standby period 
    turn_left(); // turn the car left so now it has the same reference as before
}

void avoid(){
    // if left sensor is blocked, then turn right
    if (RD2 == 0 && RD3 == 1){
        back_off();
        turn_right();
        recenter_right();
    }
    
    // if right sensor is blocked then turn left
    if(RD2 == 1 && RD3 == 0){
        back_off();
        turn_left();
        recenter_left();
    }
    
    // if both sensors are open then to turn right, although turn left will also do the job
    if(RD2 == 1 && RD3 == 1){
        back_off();
        turn_right();
        recenter_right();
    }
    
    //if both sensors are blocked, then to move backward till a path open
    if(RD2 == 0 && RD3 == 0){
        // to go back till find an open path
        while(RD2 == 0 && RD3 == 0){
            back_off();
        }
        // it go out of the loop when a path is open, so to check which path is open
        // if the right path is open, then turn right and recenter right
        if(RD3 == 1){
            turn_right();
            recenter_right();
        }
        // if the left path is open, then turn left and recenter left
        if(RD2 == 1){
            turn_left();
            recenter_left();
        }
    }
    
}

void main(void) {
    // ultra sonic pins configuration 
    TRISB1 = 0; //Trigger pin of US sensor is sent as output pin
    TRISB2 = 1; //Echo pin of US sensor is set as input pin
    
    // IR sensors pins configuration 
    TRISD2 = 1; // left sensor, defined as input pin, when low there is obstacle
    TRISD3 = 1; // right sensor, defined as input pin, when low there is obstacle
    
    // pins configuration for the motors 
    TRISC4 = 0; TRISC5 = 0; //Motor 1 (right motor) pins declared as output, Rc4=> in1 Rc5=> in2
    TRISC6 = 0; TRISC7 = 0; //Motor 2 (left motor) pins declared as output, Rc6 => in3 Rc7 =>in4
    PORTC =0;
    // pin for the forward IR sensor 
    TRISD4 = 1; // forward IR sensor, if low then object in fron of the car
    
    PWM_Init();
    
    while(1){
        /*while(start == 1){
            forward();
            __delay_ms(100);
            up = up - 100;
            calculate_distance();
            if(distance <= 5)
                avoid();
            if(up < 100 && left < 100 && right < 100){
                start = 0;
                stop();
            }
            if(up < 100){
                if(left > right){
                    up = left - right;
                    turn_left();
                }
                if(right > left){
                    up = right - left;
                    turn_right();
                }
                right = 0;
                left = 0;
            }
            
        }*/
        if(test_pwm > 200)
            test_pwm = 0;
        else
            test_pwm += 5;
        forward();
        set_DC(test_pwm);
        __delay_ms(1000);
    }
    return;
}
