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
int forward, left, right;
int duty_cycle;
int distance;
int time_takes;

// functions decleration  
void PWM_Init(int x);
void avoid();
void back_off();
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

// back_off function initialiation 
void back_off(){
    RC4=0; RC5=1; //Motor 1 reverse (right motor)
    RC6=0; RC7=1; //Motor 2 reverse (left motor)
    __delay_ms(500);
    
    RC4=0; RC5=0; //Motor 1 reverse (right motor)
    RC6=0; RC7=0; //Motor 2 reverse (left motor)
    __delay_ms(500); // stop the car, just standby period
}

//turn left function 
void turn_left(){
    RC4=1; RC5=0; //Motor 1 forward (right motor)
    RC6=0; RC7=0; //Motor 2 stop (left motor)
    __delay_ms(500);
  
    RC4 = 0; RC5 = 0; // stop the right motor also, just standby period
    __delay_ms(500);
}

// turn right function 
void turn_right(){
    RC4=0; RC5=0; //Motor 1 stop (right motor)
    RC6=1; RC7=0; //Motor 2 forward (left motor)
    __delay_ms(500);
    
    RC6 = 0; RC7 = 0; // stop the left motor also, standby period
    __delay_ms(500);
}

void main(void) {
    
    
    while(1){
        
    }
    return;
}
