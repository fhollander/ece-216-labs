/* 
 * File:   main.c
 * Author: Forest Davis-Hollander and Chunliang Tao
 *
 * Created on October 11, 2019, 9:04 AM
 * 
 *  This program reads from a 48 count encoder using change notification interrupts, and interprets the data to understand the direction and number of rotations of a motor. A pulse width modulation signal is also output from the board to turn the motor two rotations before stopping, while also displaying the degrees rotated by the motor on the LCD screen.
 */



////// these commands are to increase the clock speed to 80MHz ////////////////////
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
//////////////// end commands to increase clock speed to 80 MHz ////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include <xc.h>
#include "display_driver.h"
#include <string.h>
#include <math.h>
#define SAMPL 10 // set time
#define WAIT 200000 // set a delay time

#define COUNTS 4741.00

volatile double percent = .75;
volatile unsigned int oldD1 = 0, oldD2 = 0, newD1 = 0, newD2 = 0; // save port values
volatile int inc=0;
volatile double count=0;
volatile int rotations=0;
char value[6]; // create char array
volatile double degrees=0; // declare variable for degrees


void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) Change(void) { // INT step 1
   
    newD1 = PORTDbits.RD6; // Pin 83 is yellow wire on encoder
    newD2 = PORTDbits.RD7; // Pin 84 is yellow wire on encoder
    
    state(); // call function for state machine
    
    if(count == COUNTS){ // if we reach the positive number of counts, add a rotation
        rotations++; // increment rotations
        //count = 0; // reset count since one complete rotation is recorded for displaying 360
    }
    if(count == -COUNTS){ // if we reach negative number of counts, subtract a rotation
        rotations--; // decrement rotations
       // count = 0; // reset count since one complete rotation is recorded
    }
    
    degrees  = ((count/COUNTS)*(360)); // find the degrees
    degrees = 31.234;
    // sprintf(value,"%d",inc);  // convert char to string
    //sprintf(value,"%d",count);
    //sprintf(value,"%d",rotations);
    
    
    
    sprintf(value,"%6.6f",degrees); // convert degrees to char array
    
       
    oldD1 = newD1; // save the current values of ports for future use
    oldD2 = newD2; // save the current values of ports for future use
    
    IFS1bits.CNIF = 0; // clear the interrupt flag
}



int main(int argc, char** argv) {
    
    
    LATA = 0x0; // clear
    LATD= 0x0; // clear
    TRISA = 0xFF00; // set last 8 bits as output
    
    oldD1 = PORTDbits.RD6; // yellow
    oldD2 = PORTDbits.RD7; // white
    
    display_driver_initialize(); // call functions for display
    
    INTCONbits.MVEC = 0x1; // allow multi vector mode
    
     __builtin_disable_interrupts(); // disable interrupts
    CNCONbits.ON = 1; // turn on change notification
    CNENbits.CNEN15 = 1; // Use CN15
    CNENbits.CNEN16 = 1; // Use CN16
    CNENbits.CNEN17 = 1; // Use CN17
 
    IPC6bits.CNIP = 2; // set interrupt priority
    IPC6bits.CNIS = 0; // set interrupt subpriority
    IFS1bits.CNIF = 0; // clear the interrupt flag
    IEC1bits.CNIE = 1; // enable CN interrupt
    
    __builtin_enable_interrupts(); // enable interrupts
    
    
    while(1){ 
      
        display_driver_clear(); // clear display
        display_driver_write(value, 6); // write to display the current degrees
        
       if (rotations >= 2 || rotations <= -2){ // if rotations in either direction is more 2 or more, shut off the motor
           percent = 0; // adjust duty cycle to 0
       }

       // Pin 72 is OC1
       T2CONbits.TCKPS = 2; // Timer2 prescaler N=4 (1:4)
       PR2 = 1999; // period = (PR2+1) * N * 12.5 ns = 10 kHz
       TMR2 = 0; // initial TMR2 count is 0
       OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults

        //for duty cycle, do a simple if statement testing current value of switch1
        //then set duty cycle to (period*ADC1BUF0)/(1023) this will give a value for duty cycle based on % of potentiometer
       OC1RS = (1999*percent); // duty cycle = OC1RS/(PR2+1) = 25%   // 0< d < period
       OC1R = (1999*percent); // initialize before turning OC1 on; afterward it is read-only
       T2CONbits.ON = 1; // turn on Timer2
       OC1CONbits.ON = 1; // turn on OC1
    
      
       _CP0_SET_COUNT(0); // set count to 0
       while(_CP0_GET_COUNT() < WAIT){}; // loop for ticks time
        
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