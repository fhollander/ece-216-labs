/* 
 * File:   main.c
 * Author: Forest Davis-Hollander and Chunliang Tao
 *
 * Created on October 11, 2019, 9:04 AM
 * 
 *  This program reads from a 48 count encoder using change notification interrupts, and interprets the data to understand the direction and number of rotations of a motor. A pulse width modulation signal is also output from the board to turn the motor two rotations before stopping, while also displaying the degrees rotated by the motor on the LCD screen.
 */

//XXXYYYYYYYYYYYPROJECT_working on 12/5, saved on desktop////////////

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

volatile double percent = 0.3, percent1 = 0.3;
volatile unsigned int oldD1 = 0, oldD2 = 0, oldD3 = 0, oldD4 = 0, newD1 = 0, newD2 = 0, newD3 = 0, newD4 = 0, port1 = 1, port2 = 1, port3 = 1, port4 = 1; // save port values
volatile int inc=0, inc1=0;
volatile double count=0,count1 = 0;
char value[6]; // create char array
volatile double degrees=0, degrees1 = 0;



//Used for PID
volatile double reference = 0, reference1 = 0;
volatile double integral = 0, integral1 = 0;
volatile double previous_count = 0,previous_count1 =0 ;
volatile double current_count = 0, current_count1 = 0;
volatile double error_diff = 0, error_diff1 = 0;
volatile double error = 0, error1 = 0;
volatile double Kp = 1; //current working value for Kp =1
volatile double Ki = 0.0001; // current working value for Ki = 0.0001
volatile double ut = 0, ut1 = 0;
volatile double dt = 0;
volatile double previous_time = 0, previous_time1 = 0;
volatile double fullspeed = 0, fullspeed1 = 0;




volatile double motion = 0;



int val;
char string_value[10];
char str_left[] = "L";
char str_right[] = "R";
char str_forward[] = "F";
char str_backward[] = "B";
char str_servo_down[] = "D";
char str_servo_up[] = "U";
char str_neutral[] = "N";
volatile int select=0;

void __ISR(_UART_2_VECTOR, IPL1SOFT) IntUart2Handler(void) {
    LATAbits.LATA6 = 0;
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    if (IFS1bits.U2RXIF) { // check if interrupt generated by a RX event
        uart_read(string_value,10);
        //display_driver_clear();
        //display_driver_write(string_value, strlen(string_value));
        if(strcmp(string_value, str_left) == 0){
            LATAbits.LATA0 = 1;
            motion = -2; 
            
        }
         if(strcmp(string_value, str_right) == 0){
            LATAbits.LATA1 = 1;
            motion = 2;
        }
         if(strcmp(string_value, str_forward) == 0){
            LATAbits.LATA2 = 1;
            motion = -1;
            
        }
         if(strcmp(string_value, str_backward) == 0){
            LATAbits.LATA3 = 1;
            motion = 1;
            
        }
        if(strcmp(string_value, str_neutral) == 0){
            LATAbits.LATA3 = 1;
            motion = 0;
            
        }
        
        
    /////////////////// SERVO MOTOR CONTROL SECTION /////////////////////////
      
        if(strcmp(string_value, str_servo_down) == 0){ // if servo down command received
            select = select-1; // decrement the servo select variable
            if(select == 0){ // check if the select variable is still in correct range
                select = 1; // if not, adjust it to first servo position
            }
            else{ // otherwise, select the correct position
                 
               if(select==1){
                       char data1[6]; // GO TO +45 DEGREES
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
               if(select==2){
                       char data5[6]; // GO TO +22.5 DEGREES
                       data5[0] = 0xAA; // send AA per instructions of the Pololu protocol
                       data5[1] = 0x0C; // send default device number via Pololu protocol
                       data5[2] = 0x04; // send command byte with MSB (most significant bit) cleared
                       data5[3] = 0x00; // send start to channel 0
                       data5[4] = 0x58; // send first few data bits // originally 0x20  // new string_value 0x58 // new string_value 0x8
                       data5[5] = 0x36; // send second few data bits // originally 0x1F  // new string_value 0x36  // new string_value 0x27
                       uart_write(data5[0],2); // write via uart
                       uart_write(data5[1],2); // write via uart
                       uart_write(data5[2],2); // write via uart
                       uart_write(data5[3],2); // write via uart
                       uart_write(data5[4],2); // write via uart
                       uart_write(data5[5],2); // write via uart
               }
               if(select==3){
                       char data[6]; /// RETURN TO 0 degrees
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
               if(select==4){
                       char data52[6]; // GO TO -22.5 DEGREES
                       data52[0] = 0xAA; // send AA per instructions of the Pololu protocol
                       data52[1] = 0x0C; // send default device number via Pololu protocol
                       data52[2] = 0x04; // send command byte with MSB (most significant bit) cleared
                       data52[3] = 0x00; // send start to channel 0
                       data52[4] = 0x8; // send first few data bits // originally 0x20  // new string_value 0x58 // new string_value 0x8
                       data52[5] = 0x27; // send second few data bits // originally 0x1F  // new string_value 0x36  // new string_value 0x27
                       uart_write(data52[0],2); // write via uart
                       uart_write(data52[1],2); // write via uart
                       uart_write(data52[2],2); // write via uart
                       uart_write(data52[3],2); // write via uart
                       uart_write(data52[4],2); // write via uart
                       uart_write(data52[5],2); // write via uart
               }
               if(select==5){
                       char data51[6]; //GO TO -45 DEGREES
                       data51[0] = 0xAA; // send AA per instructions of the Pololu protocol
                       data51[1] = 0x0C; // send default device number via Pololu protocol
                       data51[2] = 0x04; // send command byte with MSB (most significant bit) cleared
                       data51[3] = 0x00; // send start to channel 0
                       data51[4] = 0x20; // send first few data bits // originally 0x20  // new string_value 0x58 // new string_value 0x8
                       data51[5] = 0x1F; // send second few data bits // originally 0x1F  // new string_value 0x36  // new string_value 0x27
                       uart_write(data51[0],2); // write via uart
                       uart_write(data51[1],2); // write via uart
                       uart_write(data51[2],2); // write via uart
                       uart_write(data51[3],2); // write via uart
                       uart_write(data51[4],2); // write via uart
                       uart_write(data51[5],2); // write via uart

               }
 
             }
             
        }
        if(strcmp(string_value, str_servo_up) == 0){ // if up command received
            select = select+1; // increase the select variable
            if(select == 6){ // check if still in range
                select = 5;
            }
            else{
               if(select==1){
                       char data1[6]; // GO TO +45 DEGREES
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
               if(select==2){
                       char data5[6]; // GO TO +22.5 DEGREES
                       data5[0] = 0xAA; // send AA per instructions of the Pololu protocol
                       data5[1] = 0x0C; // send default device number via Pololu protocol
                       data5[2] = 0x04; // send command byte with MSB (most significant bit) cleared
                       data5[3] = 0x00; // send start to channel 0
                       data5[4] = 0x58; // send first few data bits // originally 0x20  // new string_value 0x58 // new string_value 0x8
                       data5[5] = 0x36; // send second few data bits // originally 0x1F  // new string_value 0x36  // new string_value 0x27
                       uart_write(data5[0],2); // write via uart
                       uart_write(data5[1],2); // write via uart
                       uart_write(data5[2],2); // write via uart
                       uart_write(data5[3],2); // write via uart
                       uart_write(data5[4],2); // write via uart
                       uart_write(data5[5],2); // write via uart
               }
               if(select==3){
                       char data[6]; /// RETURN TO 0 degrees
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
               if(select==4){
                       char data52[6]; // GO TO -22.5 DEGREES
                       data52[0] = 0xAA; // send AA per instructions of the Pololu protocol
                       data52[1] = 0x0C; // send default device number via Pololu protocol
                       data52[2] = 0x04; // send command byte with MSB (most significant bit) cleared
                       data52[3] = 0x00; // send start to channel 0
                       data52[4] = 0x8; // send first few data bits // originally 0x20  // new string_value 0x58 // new string_value 0x8
                       data52[5] = 0x27; // send second few data bits // originally 0x1F  // new string_value 0x36  // new string_value 0x27
                       uart_write(data52[0],2); // write via uart
                       uart_write(data52[1],2); // write via uart
                       uart_write(data52[2],2); // write via uart
                       uart_write(data52[3],2); // write via uart
                       uart_write(data52[4],2); // write via uart
                       uart_write(data52[5],2); // write via uart
               }
               if(select==5){
                       char data51[6]; //GO TO -45 DEGREES
                       data51[0] = 0xAA; // send AA per instructions of the Pololu protocol
                       data51[1] = 0x0C; // send default device number via Pololu protocol
                       data51[2] = 0x04; // send command byte with MSB (most significant bit) cleared
                       data51[3] = 0x00; // send start to channel 0
                       data51[4] = 0x20; // send first few data bits // originally 0x20  // new string_value 0x58 // new string_value 0x8
                       data51[5] = 0x1F; // send second few data bits // originally 0x1F  // new string_value 0x36  // new string_value 0x27
                       uart_write(data51[0],2); // write via uart
                       uart_write(data51[1],2); // write via uart
                       uart_write(data51[2],2); // write via uart
                       uart_write(data51[3],2); // write via uart
                       uart_write(data51[4],2); // write via uart
                       uart_write(data51[5],2); // write via uart

               }
                 
             }
        }  
    /////////////// END SERVO MOTOR CONTROL SECTION /////////////////////////////////////////////////////////////
    
        IFS1bits.U2RXIF = 0; // clear the RX interrupt flag
    } 
    else if(IFS1bits.U2TXIF) { // if it is a TX interrupt
        } 
    else if(IFS1bits.U2EIF) { // if it is an error interrupt. check U3STA for reason
    } 
}


void __ISR(_TIMER_1_VECTOR, IPL1SOFT) Timer1ISR(void){ // INT step 1
   
    if (motion == -2){ //left
        reference -= 0.1;
        reference1 += 0.1;
        fullspeed = Kp*(reference - degrees);
        fullspeed1 = Kp*(reference1 - degrees1);
    }
    else if(motion == 2){
        reference += 0.1;
        reference1 -= 0.1;
        fullspeed = Kp*(reference - degrees);
        fullspeed1 = Kp*(reference1 - degrees1);
    }
    else if(motion == -1){
        reference += 0.1;
        reference1 += 0.1;
        fullspeed = Kp*(reference - degrees);
        fullspeed1 = Kp*(reference1 - degrees1);
    }
    else if (motion == 1)
    {
        reference -= 0.1;
        reference1 -= 0.1 ;
        fullspeed = Kp*(reference - degrees);
        fullspeed1 = Kp*(reference1 - degrees1);
    }
    else{
        fullspeed = 0;
        fullspeed1 = 0;
    }
    
    LATAbits.LATA4 = !LATAbits.LATA4;
    
    newD1 = PORTGbits.RG9; // Pin 14
    newD2 = PORTGbits.RG8; // Pin 12
    
    newD3 = PORTDbits.RD7; // Pin 84 is white wire on encoder
    newD4 = PORTDbits.RD6; // Pin 83 is yellow wire on encoder
    
    
    
    state(); // call function for state machine
    
    degrees  = ((count/COUNTS)*(360)); // find the degrees
    degrees1 = ((count1/COUNTS)*(360));
    
    sprintf(value,"%6.6f",degrees); // convert degrees to char array
    
       
    oldD1 = newD1; // save the current values of ports for future use
    oldD2 = newD2; // save the current values of ports for future use
    
    //These are the new encoder values
    oldD3 = newD3; 
    oldD4 = newD4;
        ///////////////// PID ////////////////////////////////
    error = reference - degrees;    // get the remaining error between reference angle and current angle
    error1 = reference1 - degrees1;
    // if over-shoot more than 5 degrees, reverse the direction
    if(error > 3){
        LATBbits.LATB0 = 0;
        LATBbits.LATB1 = 1;
        }
    else if ( error < -3) { 
        LATBbits.LATB0 = 1;
        LATBbits.LATB1 = 0;
        }
    else {
        LATBbits.LATB0 = 0;
        LATBbits.LATB1 = 0;
    }
    
    if(error1 > 3){
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 1;
        }
    else if ( error1 < -3) { 
        LATBbits.LATB2 = 1;
        LATBbits.LATB4 = 0;
        }
    else {
        LATBbits.LATB2 = 0;
        LATBbits.LATB4 = 0;
    }
 
    
    current_count = count;  
    current_count1 = count1;
    
    //Since our core timer is 8kHz. So, dt = 1/8000 s
    dt = 1/8000; 
    
    // calculate the error_diff
    error_diff = ((current_count - previous_count)/COUNTS)*360; 
    error_diff1 = ((current_count1 - previous_count1)/COUNTS)*360; 
    
    integral += error_diff * dt;    // get the integral
    integral1 += error_diff1 * dt;
    
    //Calculate the new ut
    ut = Kp*error + Ki*integral;
    ut1 = Kp*error1 + Ki*integral1;

    //get the speed
    percent = ut/fullspeed;
    percent1 = ut1/fullspeed1;
    
    previous_count = current_count;
    previous_count1 = current_count1;
    
/////////////// PID ////////////////////////////////
    
    
    IFS0bits.T1IF = 0; // clear the interrupt flag
}



int main(int argc, char** argv) {
//    LATA = 0x0; // clear
//    LATD= 0x0; // clear
//    TRISA = 0xFF00; // set last 8 bits as output
//    TRISGbits.TRISG7 = 1;
//    TRISAbits.TRISA7 = 1; // need to set this to 1 to use button S5
    
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA3 = 0;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0;
    //Set two pins as output, connecting them to IN1 and IN2 pins on the H-bridge to control the rotation direction. RB0 (PIN 25) to IN1, RB1 (PIN 24) to IN2
    TRISBbits.TRISB0 = 0;   // set RB0 (PIN 25) as output
    TRISBbits.TRISB1 = 0;   // set RB1 (PIN 24) as output
    

    TRISBbits.TRISB4 = 0;   // set RB0 (PIN 21) as output PIN 21 = PIN 24
    TRISBbits.TRISB2 = 0;   // set RB1 (PIN 23) as output PIN 20 = PIN 25
    
    
    oldD1 = PORTGbits.RG9; // Pin 14 is white wire on encoder
    oldD2 = PORTGbits.RG8; // Pin 12 is yellow wire on encoder
    
    oldD3 = PORTDbits.RD7; // Pin 84
    oldD4 = PORTDbits.RD6; // Pin 83
    
    
    DDPCONbits.JTAGEN = 0; //clear data debug control reg
    //TRISAbits.TRISA6 = 0;
    //LATA = 0; //LEDs are off (debug purposes)
    
    
    display_driver_initialize(); //initialize LCD
    display_driver_clear();//clear contents of LCD
    
    uart_initialize(2);//initialize UART2, will need to set the baud rate to 10 (to get 100 total samples in 10 sec)
    
    int i;
    int val;
    char string_value[10];

    

    
    INTCONbits.MVEC = 1; // allow multi vector mode
    
     __builtin_disable_interrupts(); // disable interrupts 
     
    U2MODEbits.UEN = 0;
    // configure the UART interrupts
    U2STAbits.URXISEL = 0x0; // RX interrupt when receive buffer not empty
    IFS1bits.U2RXIF = 0; // clear the rx interrupt flag. for
    // tx or error interrupts you would also need to clear
    // the respective flags
    IPC8bits.U2IP = 1; // interrupt priority
    IPC8bits.U2IS = 1; // interrupt subpriority
    IEC1bits.U2RXIE = 1; // enable the RX interrupt
    
    // turn on UART1
    U2MODEbits.ON = 1;

    //Timer1: 8kHz
    PR1 = 10000; // set period register
    TMR1 = 0; // initialize count to 0
    T1CONbits.TCKPS = 0; // set prescaler to 256
    T1CONbits.TGATE = 0; // not gated input (the default)
    T1CONbits.TCS = 0; // PCBLK input (the default)
    T1CONbits.ON = 1; // turn on Timer1
    IPC1bits.T1IP = 1; // INT step 4: priority
    IPC1bits.T1IS = 0; // subpriority
    IFS0bits.T1IF = 0; // INT step 5: clear interrupt flag
    IEC0bits.T1IE = 1;
    
    __builtin_enable_interrupts(); // enable interrupts
    _CP0_SET_COUNT(0);
    
    while(1){ 
      
        display_driver_clear(); // clear display
        display_driver_write(value, 6); // write to display the current degrees


       // Pin 72 is OC1
       T2CONbits.TCKPS = 2; // Timer2 prescaler N=4 (1:4)
       PR2 = 1999; // period = (PR2+1) * N * 12.5 ns = 10 kHz
       TMR2 = 0; // initial TMRC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults2 count is 0
       OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
       OC4CONbits.OCM = 0b110;
        //for duty cycle, do a simple if statement testing current value of switch1
        //then set duty cycle to (period*ADC1BUF0)/(1023) this will give a value for duty cycle based on % of potentiometer
       OC1RS = (1999*percent); // duty cycle = OC1RS/(PR2+1) = 25%   // 0< d < period
       OC1R = (1999*percent); // initialize before turning OC1 on; afterward it is read-only
       
       OC4RS = (1999*percent1); // duty cycle = OC1RS/(PR2+1) = 25%   // 0< d < period
       OC4R = (1999*percent1); // initialize before turning OC1 on; afterward it is read-only
       T2CONbits.ON = 1; // turn on Timer2
       OC1CONbits.ON = 1; // turn on OC1     
       OC4CONbits.ON = 1;
    }
   
    return 0 ;
}



    int state(){ /// function for state machine
    // state 1
        if(oldD1 == 0 && oldD2 == 0){   // if start is 00
            if(newD1 == 0 && newD2 == 0){  // read new port values
                inc =00; // stays in place
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
    
        
        
        
    // state 2
        if(oldD3 == 0 && oldD4 == 0){   // if start is 00
            if(newD3 == 0 && newD4 == 0){  // read new port values
                inc1 =00; // strays in place
            }
            if(newD3 == 0 && newD4 == 1){ // read new port values
                inc1 =01; // + direction
                count1++; // increment count
                
            }   
            if(newD3 == 1 && newD4 == 0){ // read new port values
                 inc1 =-1; // - direction
                 count1--;  // decrease count

            }   
            
        }
        if(oldD3 == 0 && oldD4 == 1){ // if start is 01
            if(newD3 == 0 && newD4 == 0){ // read new port values
               inc1 =-1; // - direction
               count1--; // decrease count
            }
            if(newD3 == 0 && newD4 == 1){ // read new port values
                 inc1 =00;
            }   
            if(newD3 == 1 && newD4 == 1){ // read new port values
                inc1 =01;
                count1++; // increment count
            } 
        }   
        if(oldD3 == 1 && oldD4 == 0){   /// if start is 10
            if(newD3 == 0 && newD4 == 0){ // read new port values
               inc1 =01; // + direction
               count1++; // increment count
            }
            if(newD3 == 1 && newD4 == 1){// read new port values
                inc1 =-1; 
                count1--; // decrease count
            }  
            if(newD3 == 1 && newD4 == 0){// read new port values
                 inc1 =00; 

            }   
            
        }   
        if(oldD3 == 1 && oldD4 == 1){ // if start is 11
            if(newD3 == 1 && newD4 == 0){ // read new port values
                 inc1 =01;  // + direction
                 count1++;  // increment count
            }   
            if(newD3 == 0 && newD4 == 1){// read new port values
                 inc1 =-1; // - direction
                 count1--; // decrease count
            }   
            if(newD3 == 1 && newD4 == 1){// read new port values
                 inc1 =00; 
            }   
        }  
        
        
        
        
        
        
        
        
    
    return 0;
}