/* 
 * File:   main.c
 * Author: Forest Davis-Hollander and Chunliang Tao
 *
 * Created on November 15, 2019
 * 
 * This program uses UART2 to communicate with a servo motor controller. The program first initializes the desired UART2 and then continuously loops until a change notification is detected. Depending on which button was pressed, the servo motor will rotate to either the home position, 45 degrees, or -45 degrees.
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

volatile double delay_diff = 30000;
volatile unsigned int oldD1 = 0, oldD2 = 0, newD1 = 0, newD2 = 0, port1 = 1, port2 = 1, port3 = 1, port4 = 1; // save port values



void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) Change(void) { // INT step 1
    
    //Read the changed value of monitored port
 
    port1 = PORTDbits.RD6; // read from ports
    port2 = PORTDbits.RD7;// read from ports
    
    port3 = PORTGbits.RG7;// read from ports

    /////// for changing rotations ///////// monitor each push button
    
        if(port3 == 0){   // S5 (PIN 92) decreases the angle by 45 degrees
 
            char data5[6]; // initialize array to hold hex values
            data5[0] = 0xAA; // send AA per instructions of the Pololu protocol
            data5[1] = 0x0C; // send default device number via Pololu protocol
            data5[2] = 0x04; // send command byte with MSB (most significant bit) cleared
            data5[3] = 0x00; // send start to channel 0
            data5[4] = 0x20; // send first few data bits
            data5[5] = 0x1F; // send second few data bits
            uart_write(data5[0],2); // write via uart
            uart_write(data5[1],2); // write via uart
            uart_write(data5[2],2); // write via uart
            uart_write(data5[3],2); // write via uart
            uart_write(data5[4],2); // write via uart
            uart_write(data5[5],2); // write via uart
       
            }
 
      
        if(port1 == 0){ //S3 (PIN 83) return to home 0 degrees 
            
            char data[6];
            data[0] = 0xAA; // send AA per instructions of the Pololu protocol
            data[1] = 0x0C; // send default device number via Pololu protocol
            data[2] = 0x04; // send command byte with MSB (most significant bit) cleared
            data[3] = 0x00; // send start to channel 0
            data[4] = 0x70; // send first few data bits
            data[5] = 0x2E; // send second few data bits
            uart_write(data[0],2); // write via uart
            uart_write(data[1],2); // write via uart
            uart_write(data[2],2); // write via uart
            uart_write(data[3],2); // write via uart
            uart_write(data[4],2); // write via uart
            uart_write(data[5],2); // write via uart

            }
        if(port2 == 0){ // S6 (PIN 84) increases the  angle by 45 degrees
            
             char data1[6];
            data1[0] = 0xAA; // send AA per instructions of the Pololu protocol
            data1[1] = 0x0C; // send default device number via Pololu protocol
            data1[2] = 0x04; // send command byte with MSB (most significant bit) cleared
            data1[3] = 0x00; // send start to channel 0
            data1[4] = 0x40; // send first few data bits
            data1[5] = 0x3E; // send second few data bits
            uart_write(data1[0],2); // write via uart
            uart_write(data1[1],2); // write via uart
            uart_write(data1[2],2); // write via uart
            uart_write(data1[3],2); // write via uart
            uart_write(data1[4],2); // write via uart
            uart_write(data1[5],2); // write via uart
        }

    IFS1bits.CNIF = 0;  // clear the interrupt flag
}




int main(int argc, char** argv) {

    LATA = 0x0; // clear
    LATD= 0x0; // clear

    // P11 to P92 for button S5 to CN9
    
    
    uart_initialize(2); // initialize UART2
   
   
    INTCONbits.MVEC = 0x1; // allow multi vector mode
    
     __builtin_disable_interrupts(); // disable interrupts  
    CNCONbits.ON = 1; //  turn on CN 
    CNENbits.CNEN15 = 1; // Use CN15 for Button S3
    CNENbits.CNEN16 = 1; // Use CN16 for Button S6
    CNENbits.CNEN9 = 1; // Use CN9 for Button S5 pin 1 RG7

    IPC6bits.CNIP = 3; // set interrupt priority
    IPC6bits.CNIS = 0; // set interrupt subpriority
    IFS1bits.CNIF = 0; //  clear the interrupt flag
    IEC1bits.CNIE = 1; //  enable CN interrupt     
    
    __builtin_enable_interrupts(); // enable interrupts
     
    while(1){ // loop forever 
    }
    return (EXIT_SUCCESS);
}

