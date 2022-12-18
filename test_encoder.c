/*
 * File:   test_encoder.c
 * Author: shaarawy
 *
 * Created on December 15, 2022, 1:36 AM
 */

// CONFIG
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 4000000
#include <xc.h>

int right;
int left;

// functions decleration  
void PWM_Init(void);
void PWM_right_init(void);
void set_DC(int x);
void set_DC_right(int x);
void forward(void);
void stop(void);
void turn_right(void);

// PMW function
void PWM_Init(void)
{
    //////////////// the function is tested and it is working //////////////////////
    // for this config the max DC is 100 which gives full power
  //--[ Configure The CCP Module For PWM Mode ]--
  CCP1M3 = 1;
  CCP1M2 = 1;
  TRISC2 = 0; // The CCP1 Output Pin (PWM)
  // Set The PWM Frequency (10kHz)
  PR2 = 24;
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
   // for this config the max DC is 100 which gives full power
  //--[ Configure The CCP Module For PWM Mode ]--
  CCP2M3 = 1;
  CCP2M2 = 1;
  TRISC1 = 0; // The CCP1 Output Pin (PWM)
  // Set The PWM Frequency (10kHz)
  PR2 = 24;
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

// forward function 
void forward(void){
    RC4=1; RC5=0; //Motor 1 forward (right motor)
    RC6=1; RC7=0; //Motor 2 forward (left motor)
}

// stop function to stop the car
void stop(void){
    RC4 = 0; RC5 = 0; // Motor 1 stop (right motor)
    RC6=0; RC7=0; //Motor 2 stop (left motor)
}

// turn right function 
void turn_right(void){
    RC4=0; RC5=0; //Motor 1 stop (right motor)
    RC6=1; RC7=0; //Motor 2 forward (left motor)
    //__delay_ms(500);
    
    //stop(); // stop the car, just standby period
    //__delay_ms(500);
}


void main(void) {
    
    right = 0;
    left = 0;
    
    // pins configuration for the motors 
    TRISC4 = 0; TRISC5 = 0; //Motor 1 (right motor) pins declared as output, Rc4=> in1 Rc5=> in2
    TRISC6 = 0; TRISC7 = 0; //Motor 2 (left motor) pins declared as output, Rc6 => in3 Rc7 =>in4
    RC4 = 0;
    RC5 = 0;
    RC6 = 0;
    RC7 = 0;
         //global interrupt enable
    TRISB0 = 1;   //setup RB0 as input
    //RB0 = 0;
    //INTF = 0; //clear interrupt flag bit
    
    INTEDG = 1; '  //configure detection of interrupt on rising edge
    INTE = 1;
    GIE = 1;
    

    
    TRISB4 = 1;
    TRISB5 = 1;
    RB4 = 0;
    RB5 = 0;
    
    TRISD3 = 0;
    RD3 = 0;
    
    PWM_Init();
    set_DC(60);//left wheel faster 
    PWM_right_init();
    set_DC_right(0 );//right wheel slower 
    
    
    while(1){
        forward();
        RD3 = 1;
        
        if(right == 21 || left == 21){
            stop();
            RD3 = 0;
            __delay_ms(3000);
            right = 0;
            left = 0;
        }
    }
    
    return;
}

void __interrupt() ISR(void){
    
    //Makes sure that it is PORTB On-Change Interrupt
    if(INTF == 1){
        //if(RB4 == 1)
          //  right = right + 1;
        //if(RB5 == 1)
          //  left = left + 1;
        //RBIF = 0;
        right = right + 1;
        left = left + 1;
        INTF=0;
    }
    
}
