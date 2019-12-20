/* 
 * File:   main.c
 * Author: Forest Davis-Hollander and Chunliang Tao
 *
 * Created on November 14, 2019
 * 
 * This program uses UART2 to communicate with a workstation using an external Matlab script. The program receives a series of ASCII values corresponding to either "Overdamped" or Underdamped," and then it activates the motor. While the motor is running, the state machine function from the previous lab reads from the encoder on the motor, and a the current angle is determined and sent over UART2 back to the workstation. Additionally, the PI controller from the last lab will cause the motor to move to a reference angle with either overdamped or underdamped behavior, depending on the Ki and Kp values specified.
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
#include "uart_interface.h"
#include "display_driver.h"
#include <string.h>
#include <math.h>


#define SAMPL 10 // set time
#define WAIT 200000 // set a delay time
#define COUNTS 4741.00 // total counts of the encoder

volatile double percent = 0.7;
volatile unsigned int oldD1 = 0, oldD2 = 0, newD1 = 0, newD2 = 0, port1 = 1, port2 = 1, port3 = 1, port4 = 1; // save port values
volatile int inc=0;
volatile double count=0;
volatile int rotations=0;
char value[6]; // create char array
volatile double degrees=0, degrees_old;
volatile int motor_dir = 1; // determine positive or negative direction  (change direction for M1IN1 of H bridge)
volatile int motor_dir1 = 0; // determine positive or negative direction   (change direction for M1IN2 of H bridge)
volatile double degrees_max = 0; // set max degrees to rotate


//Used for PID
volatile double reference = 0; // create reference angle
volatile double integral = 0; // create integral variable
volatile double previous_count = 0; // record previous count
volatile double current_count = 0; // record current count
volatile double error_diff = 0; // record difference in error
volatile double error = 0; // record error
volatile double Kp = 1000; //current working value for Kp =1
volatile double Ki = 0.0001; // current working value for Ki = 0.0001
volatile double ut = 0;  // make ut variable for PI 
volatile double dt = 0; // make small change in time
volatile double previous_time = 0; // get previous clock time
volatile double fullspeed = 0; // set speed of motor


volatile double delay1 = 0;
volatile double delay2 = 0;
volatile double delay_diff = 30000;

int increment = 0;
int x =0;

volatile int write = 0; // create write variable to determine if timer interrupt for UART should write or not
volatile int write_count = 0; // initialize count for UART writes (max 100)



void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1ISR(void){ // INT step 1
 
    newD1 = PORTGbits.RG9; // Pin 14 is yellow wire on encoder
    newD2 = PORTGbits.RG8; // Pin 12 is white wire on encoder
    
    state(); // call function for state machine
    
    degrees  = ((count/COUNTS)*(360)); // find the degrees
    sprintf(value,"%.1f",degrees); // convert degrees to char array
       
    oldD1 = newD1; // save the current values of ports for future use
    oldD2 = newD2; // save the current values of ports for future use

     ///////////////// PID ////////////////////////////////
    error = reference - degrees;    // get the remaining error between reference angle and current angle
    
    // if over-shoot more than 5 degrees, reverse the direction
    if(error > 1){ // in one direction
        LATBbits.LATB0 = 0;
        LATBbits.LATB1 = 1;
    }
    else if ( error < -1) { // in another direction
        LATBbits.LATB0 = 1;
        LATBbits.LATB1 = 0;
    }
    else { // within 1 degree then stop the motor
        percent = 0;
    }
    current_count = count;    
    //Since our core timer is 8kHz. So, dt = 1/8000 s
    dt = 1/8000; 
    // calculate the error_diff
    error_diff = ((current_count - previous_count)/COUNTS)*360; 
    integral += error_diff * dt;    // get the integral
    
    //Calculate the new ut
    ut = Kp*error + Ki*integral;
    //get the speed
    percent = ut/fullspeed;
    previous_count = current_count; // set old count to new count
    
    OC1RS = (1999*percent); // duty cycle = OC1RS/(PR2+1) = 25%   // 0< d < period
    OC1R = (1999*percent); // initialize before turning OC1 on; afterward it is read-only
///////////////// PID ////////////////////////////////
    
    IFS0bits.T1IF = 0; // clear the interrupt flag
}


void __ISR(_TIMER_3_VECTOR, IPL2SOFT) Timer3ISR(void){ // INT step 1
   
    char end [] = "\r\n";
    strcat(value, end); // add \r\n to make sure scanf stops as we want
    
    if(write==1){ // so UART can write only when it has been given the command to do so
        int j;
        for(j=0; j<10; j++){ // write each character
            if(value[j] == '.'){ // if it reaches a period, exit since matlab can't read periods in integers
                uart_write(end[1],2); // write \n
                break; // exit loop, done transmitting
            }
            else{
            uart_write(value[j],2); // write each character in value (which are the degrees)
               //for(increment= 0; increment<WAIT; increment++){}
            }
        }
            
        write_count++; // increment count of writes

        if(write_count == 100){ // determine number of transmissions
            IEC0bits.T3IE = 0; // reset flag to turn off this timer interrupt
            write =0; // reset write to 0 to stop writing UART
            write_count = 0; // reset write count
        }
   }
    
    IFS0bits.T3IF = 0; // clear the interrupt flag
}






int main(int argc, char** argv) {
    
   LATA = 0x0; // clear
   LATD= 0x0; // clear
   TRISA = 0xFF00; // set last 8 bits as output
   TRISAbits.TRISA7 = 1; // need to set this to 1 to use button S5
   TRISGbits.TRISG7 = 1;
   
   oldD1 = PORTGbits.RG9; // Pin 14 is yellow wire on encoder
   oldD2 = PORTGbits.RG8; // Pin 12 is white wire on encoder

    //Set two pins as output, connecting them to IN1 and IN2 pins on the H-bridge to control the rotation direction. RB0 (PIN 25) to IN1, RB1 (PIN 24) to IN2
    TRISBbits.TRISB0 = 0;   // set RB0 (PIN 25) as output
    TRISBbits.TRISB1 = 0;   // set RB1 (PIN 24) as output
 
    
   display_driver_initialize(); // initialize display driver
   uart_initialize(2); // initialize UART2

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
    
    //Timer3
    PR3 = 31249; // set period register for 10Hz
    TMR3 = 0; // initialize count to 0
    T3CONbits.TCKPS = 7; // set prescaler to 256
    T3CONbits.TGATE = 0; // not gated input (the default)
    T3CONbits.TCS = 0; // PCBLK input (the default)
    T3CONbits.ON = 1; // turn on Timer3
    IPC3bits.T3IP = 2; // interrupt priority
    IPC3bits.T3IS = 0; // subpriority
    IFS0bits.T3IF = 0; //  clear interrupt flag
    IEC0bits.T3IE = 0; // disable timer interrupt to start
    
    PR1 = 10000; // set period register
    TMR1 = 0; // initialize count to 0
    T1CONbits.TCKPS = 0; // set prescaler to 256
    T1CONbits.TGATE = 0; // not gated input (the default)
    T1CONbits.TCS = 0; // PCBLK input (the default)
    T1CONbits.ON = 1; // turn on Timer1
    IPC1bits.T1IP = 2; //  priority
    IPC1bits.T1IS = 0; // subpriority
    IFS0bits.T1IF = 0; // clear interrupt flag
    IEC0bits.T1IE = 1; // turn on interrupt

    __builtin_enable_interrupts(); // enable interrupts



    int sum =0;
    //char value [20] ={0};
    char nvalue [20];
    
    
    while(1) {
        display_driver_clear(); // clear display
        display_driver_write(value, 6); // write degrees to display
        
       
        if(write==0){ // as long as write is still 0
            sum=uart_read(2) + sum; // read UART2 and sum values
            
            // UNDERDAMPED should have ASCII sum of 809
            // set reference angle, set coefficient
             if(sum==809){ // if equals the ASCII value of UNDERDAMPED
                write = 1; // set write to allow UART to transmit
                reference += 180; // increase reference angle
                fullspeed = (reference - degrees); // calculate new fullspeed
                Kp = 10; //current working value for Kp =1 // WANT TO BE UNDERDAMPED
                Ki = 0.1; // current working value for Ki = 0.0001

                IEC0bits.T3IE = 1; // enable timer interrupt
                IEC0bits.T1IE = 1;// enable timer interrupt
                sum = 0;
            }
            
            // OVERDAMPED should have ASCII sum of 743
            // set reference angle, set coefficient
            if(sum==743){// if equals the ASCII value of OVERDAMPED
                write = 1; // set write to allow UART to transmit
                reference += 180; // increase reference angle
                fullspeed = (reference - degrees); // calculate new fullspeed
                Kp = 1; //current working value for Kp =1 // WANT TO BE OVERDAMPED
                Ki = 0.0001; // current working value for Ki = 0.0001

                IEC0bits.T3IE = 1; // enable timer interrupt
                IEC0bits.T1IE = 1; // enable timer interrupt
                sum = 0;
            }
  
           sprintf(value,"%d",sum); // convert degrees to char array

          
               }
        if(write==1){ // if UART is currently writing
            
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

        }

       
    }
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