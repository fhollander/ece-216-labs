/* 
 * File:   main.c
 * Author: Forest Davis-Hollander and Chunliang Tao
 *
 * Created on October 31, 2019
 * 
 * This program reads from a 48 count encoder using change notification interrupts, and interprets the data to understand the direction and number of rotations of a motor. A pulse width modulation signal is also output from the board to turn the motor two rotations before stopping, while also displaying the degrees rotated by the motor on the LCD screen. This program uses a basic PI controller to determine the duty cycle of the PWM signal based on the desired reference angle, allowing the motor to stop within 10 degrees of the reference angle. It further uses the push buttons to change the direction and number of degrees rotated by the motor.
 */



////// These commands are to increase the clock speed to 80MHz ////////////////////
#pragma config DEBUG = OFF // Background Debugger disabled
#pragma config FPLLMUL = MUL_20 // PLL Multiplier: Multiply by 20
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider: Divide by 2
#pragma config FPLLODIV = DIV_1 // PLL Output Divider: Divide by 1

#pragma config FWDTEN = OFF // WD timer: OFF
#pragma config WDTPS = PS4096 // WD period: 4.096 sec
#pragma config POSCMOD = HS // Primary Oscillator Mode: High Speed xtal
#pragma config FNOSC = PRIPLL // Oscillator Selection: Primary oscillator

#pragma config FPBDIV = DIV_1 // Peripheral Bus Clock: Divide by 1
#pragma config UPLLEN = ON // USB clock uses PLL
#pragma config UPLLIDIV = DIV_2
//////////////// End commands to increase clock speed to 80 MHz ////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <xc.h>
#include "display_driver.h"
#include <string.h>
#include <math.h>
#define SAMPL 10 // set time
#define WAIT 200000 // set a delay time
#define COUNTS 4741.00 // set counts constant

volatile double percent = 0.3; // initial motor speed is 30%
volatile unsigned int oldD1 = 0, oldD2 = 0, newD1 = 0, newD2 = 0, port1 = 1, port2 = 1, port3 = 1, port4 = 1; // save port values
volatile int inc=0;
volatile double count=0; // variable to keep track of encoder count for angle tracking
volatile int rotations=0;
char value[6]; // create char array
volatile double degrees=0, degrees_old;
volatile int motor_dir = 1; // determine positive or negative direction  (change direction for M1IN1 of H bridge)
volatile int motor_dir1 = 0; // determine positive or negative direction   (change direction for M1IN2 of H bridge)
volatile double degrees_max = 0; // set max degrees to rotate


//Used for PID
volatile double reference = 0; // create reference angle variable
volatile double integral = 0; // create integral variable
volatile double previous_count = 0; // record previous count of encoder
volatile double current_count = 0; // record current count of encoder
volatile double error_diff = 0; // error difference variable
volatile double error = 0; // error variable
volatile double Kp = 1; //current working value for Kp =1
volatile double Ki = 0.0001; // current working value for Ki = 0.0001
volatile double ut = 0; 
volatile double dt = 0; //time difference
volatile double previous_time = 0; // previous time variable
volatile double fullspeed = 0; // define motor speed

volatile double delay1 = 0; // use to avoid mutliple ISR calls in same push
volatile double delay2 = 0;
volatile double delay_diff = 30000;

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1ISR(void){ // INT step 1
   
    newD1 = PORTGbits.RG9; // Pin 14 is yellow wire on encoder
    newD2 = PORTGbits.RG8; // Pin 12 is white wire on encoder
    
    state(); // call function for state machine
    
    
    degrees  = ((count/COUNTS)*(360)); // find the degrees
    sprintf(value,"%6.6f",degrees); // convert degrees to char array
    
       
    oldD1 = newD1; // save the current values of ports for future use
    oldD2 = newD2; // save the current values of ports for future use

    
    ///////////////// PID START ////////////////////////////////
    error = reference - degrees;    // get the remaining error between reference angle and current angle
    
    if(error > 5){
        LATBbits.LATB0 = 0; // set direction pins for output to control motor
        LATBbits.LATB1 = 1; // set direction pins for output to control motor
    }
    else if ( error < -5) { 
        LATBbits.LATB0 = 1; // set direction pins for output to control motor
        LATBbits.LATB1 = 0; // set direction pins for output to control motor
    }
    else {
        LATBbits.LATB0 = 0; // set direction pins for output to control motor
        LATBbits.LATB1 = 0; // set direction pins for output to control motor
    }
    current_count = count; // set current count to count value
    
    
    dt = 1/8000; //Since core timer is 8kHz. So, dt = 1/8000 s
   
    error_diff = ((current_count - previous_count)/COUNTS)*360;  // calculate the error_diff
    integral += error_diff * dt;    // get the integral value
    
    
    ut = Kp*error + Ki*integral; //Calculate the new ut
    
    percent = ut/fullspeed; //calculate new speed based on a percent
    
    previous_count = current_count; // set previous count variable to current count
    
    /////////////// PID END ////////////////////////////////
    
    
    IFS0bits.T1IF = 0; // clear the interrupt flag
}


void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) Change(void) { // INT step 1
    
    //Read the changed value of monitored port
 
    port1 = PORTDbits.RD6; // read from ports
    port2 = PORTDbits.RD7;// read from ports
    port4 = PORTDbits.RD13;// read from ports
    port3 = PORTGbits.RG7;// read from ports

    delay2 = _CP0_GET_COUNT();
    delay_diff = delay2 - delay1; // use this to avoid problem with push button calling interrupts too many times
    /////// for changing rotations, monitor each push button
    if (delay_diff > 20000000){ // use this delay to avoid multiple ISR calls
        if(port3 == 0){   // S5 (PIN 92) increases the reference angle by 90 degrees
            LATAbits.LATA4 = 1; // light led to prove that push button interrupt succeeds
            int i;
            for(i=0;i<100000;i++){    
            }
            LATAbits.LATA4 = 0; 
            reference += 90; // increase reference angle
            fullspeed = Kp*(reference - degrees); // calculate new fullspeed
            delay1 = _CP0_GET_COUNT(); // get clock ticks to check if enough time has elapsed from button push
            }
        if(port4 == 0){  // S4 (PIN 80) decreases the reference angle by 90 degrees.

            LATAbits.LATA6 = 1; // light led to prove that push button interrupt succeeds
            int i;
            for(i=0;i<100000;i++){    
            }
            LATAbits.LATA6 = 0; 
            reference -= 90; // decrease reference angle
            fullspeed = Kp*(reference - degrees); // calculate new fullspeed
            delay1 = _CP0_GET_COUNT(); // get clock ticks to check if enough time has elapsed from button push
            }
        if(port1 == 0){ //S3 (PIN 83) increases the reference angle by 45 degrees 
            LATAbits.LATA2 = 1; // light led to prove that push button interrupt succeeds
            int i;
            for(i=0;i<100000;i++){    
            }
            LATAbits.LATA2 = 0;
            reference += 45; // increase reference angle
            fullspeed = Kp*(reference - degrees); // calculate new fullspeed
            delay1 = _CP0_GET_COUNT(); // get clock ticks to check if enough time has elapsed from button push
            }
        if(port2 == 0){ // S6 (PIN 84) decreases the reference angle by 45 degrees
            LATAbits.LATA3 = 1; // light led to prove that push button interrupt succeeds
            int i;
            for(i=0;i<100000;i++){
            }
            LATAbits.LATA3 = 0;
            reference -= 45; // decrease reference angle
            fullspeed = Kp*(reference - degrees); // calculate new fullspeed
            delay1 = _CP0_GET_COUNT(); // get clock ticks to check if enough time has elapsed from button push
        }}
    else{
        
    }
     
    IFS1bits.CNIF = 0;  // clear the interrupt flag
}

int main(int argc, char** argv) {
    
    
    //Required jumpers:
    //Connect white encoder output wire to P14
    //Connect yellow encoder output wire to P12
    //Connect M1IN1 to P25
    //Connect M1IN2 to P24
    //Connect P11 to P92 for CN interrupt for S5
    
    //**Note: button presses must be done quickly, otherwise the ISR will be called multiple times!
 
    
    LATA = 0x0; // clear
    LATD= 0x0; // clear
    TRISA = 0xFF00; // set last 8 bits as output
    TRISGbits.TRISG7 = 1;
    TRISAbits.TRISA7 = 1; // need to set this to 1 to use button S5
    
    //Set two pins as output, connecting them to IN1 and IN2 pins on the H-bridge to control the rotation direction. RB0 (PIN 25) to IN1, RB1 (PIN 24) to IN2
    TRISBbits.TRISB0 = 0;   // set RB0 (PIN 25) as output
    TRISBbits.TRISB1 = 0;   // set RB1 (PIN 24) as output
 
    oldD1 = PORTGbits.RG9; // Pin 14 is yellow wire on encoder
    oldD2 = PORTGbits.RG8; // Pin 12 is white wire on encoder
    
    display_driver_initialize(); // call functions for display
    
    INTCONbits.MVEC = 0x1; // allow multi vector mode
    
     __builtin_disable_interrupts(); // disable interrupts  
    CNCONbits.ON = 1; //  turn on CN 
    CNENbits.CNEN15 = 1; // Use CN15 for Button S3
    CNENbits.CNEN16 = 1; // Use CN16 for Button S6
    CNENbits.CNEN9 = 1; // Use CN9 for Button S5 pin 1 RG7
    CNENbits.CNEN19 = 1; // Use CN19 for Button S4

    IPC6bits.CNIP = 3; // set interrupt priority
    IPC6bits.CNIS = 0; // set interrupt subpriority
    IFS1bits.CNIF = 0; //  clear the interrupt flag
    IEC1bits.CNIE = 1; //  enable CN interrupt     
     
    //Timer1: 8kHz
    PR1 = 10000; // set period register
    TMR1 = 0; // initialize count to 0
    T1CONbits.TCKPS = 0; // set prescaler to 256
    T1CONbits.TGATE = 0; // not gated input
    T1CONbits.TCS = 0; // deffault timer input
    T1CONbits.ON = 1; // turn on Timer1
    IPC1bits.T1IP = 2; // interrupt priority
    IPC1bits.T1IS = 0; // subpriority
    IFS0bits.T1IF = 0; //  clear interrupt flag
    IEC0bits.T1IE = 1;
    
    __builtin_enable_interrupts(); // enable interrupts
    _CP0_SET_COUNT(0);
    
    while(1){ 
      
       display_driver_clear(); // clear display
       display_driver_write(value, 6); // write to display the current degrees

       // Pin 72 is OC1
       T2CONbits.TCKPS = 2; // Timer2 prescaler N=4 (1:4)
       PR2 = 1999; // period = (PR2+1) * N * 12.5 ns = 10 kHz
       TMR2 = 0; // initial TMR2 count is 0
       OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
       OC1RS = (1999*percent); // change duty cycle based on the percent 0-100
       OC1R = (1999*percent); // initialize before turning OC1 on; afterward it is read-only
       T2CONbits.ON = 1; // turn on Timer2
       OC1CONbits.ON = 1; // turn on OC1     
    }
   
    return 0 ;
}



    int state(){ /// function for state machine
    
        if(oldD1 == 0 && oldD2 == 0){   // if start is 00
            if(newD1 == 0 && newD2 == 0){  // read new port values
                inc =00; // strays in place
            }
            if(newD1 == 0 && newD2 == 1){ // read new port values
                inc =01; // + direction
                count++; // increment count
            }   
            if(newD1 == 1 && newD2 == 0){ // read new port values
                 inc =-1; // - direction
                 count--;  // decrease count
            }
        }
        if(oldD1 == 0 && oldD2 == 1){ // if start is 01
            if(newD1 == 0 && newD2 == 0){ // read new port values
               inc =-1; // - direction
               count--; // decrease count
            }
            if(newD1 == 0 && newD2 == 1){ // read new port values
                 inc =00;
            }   
            if(newD1 == 1 && newD2 == 1){ // read new port values
                inc =01;
                count++; // increment count
            } 
        }   
        if(oldD1 == 1 && oldD2 == 0){   /// if start is 10
            if(newD1 == 0 && newD2 == 0){ // read new port values
               inc =01; // + direction
               count++; // increment count
            }
            if(newD1 == 1 && newD2 == 1){// read new port values
                inc =-1; 
                count--; // decrease count
            }  
            if(newD1 == 1 && newD2 == 0){// read new port values
                 inc =00;
            }
        }   
        if(oldD1 == 1 && oldD2 == 1){ // if start is 11
            if(newD1 == 1 && newD2 == 0){ // read new port values
                 inc =01;  // + direction
                 count++;  // increment count
            }   
            if(newD1 == 0 && newD2 == 1){// read new port values
                 inc =-1; // - direction
                 count--; // decrease count
            }   
            if(newD1 == 1 && newD2 == 1){// read new port values
                 inc =00; 
            }   
        }  
    
    return 0;
}
