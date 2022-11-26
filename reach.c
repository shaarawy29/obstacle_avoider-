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
int time_takes;

// functions decleration  
void PWM_Init(int x);
void avoid();
void back_off();
void forward();
void stop();
void recentre_left();
void recentre_right();
void turn_left();
void turn_right();


// function initialization 

// PMW function
void PWM_Init(int x)
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

// back_off function initialiation 
void back_off(){
    RC4=0; RC5=1; //Motor 1 reverse (right motor)
    RC6=0; RC7=1; //Motor 2 reverse (left motor)
    __delay_ms(500);
    
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

// recentre left function 
void recentre_left(){
    while(RD2 == 1){
        forward();
    }
    stop(); // stop the car for standby period then recentre the car
    turn_right(); // turn the car right so now it has the same reference 
    
}

// recentre right funciton, to recentere the car after turning right
void recentre_right(){
    while(RD3 == 1){
        forward();
    }
    stop(); // standby period 
    turn_left(); // turn the car left so now it has the same reference as before
}

void main(void) {
    
    
    while(1){
        
    }
    return;
}
