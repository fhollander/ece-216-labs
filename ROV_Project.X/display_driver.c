/* 
 * File:   main.c
 * Author: Forest Davis-Hollander
 *
 * Created on September 13, 2019, 9:07 AM
 * This program is the controls the subroutines that are needed in the main program. This contains the code for the display_driver_enable, display_driver_clear, display_driver_initialize, and display_driver_write subroutines. Each subroutine is in charge of their respective part of writing to the LCD.
 */


#include "display_driver.h"
#include <xc.h>


void display_driver_enable(){
    int x;
    for(x=0; x<2000; x++){ // continue to loop
    }
    TRISDbits.TRISD4 = 0; // set bits 4 as outputs (for enable))
    TRISDbits.TRISD5 = 0; // is this true?
    
    LATDbits.LATD5 = 0; // is this necessary?
    LATDbits.LATD4 = 1; // set enable bit (RD4) to high 
    
    for(x=0; x<2000; x++){ // continue to loop
    }
    LATDbits.LATD4 = 0; // set enable bit (RD4) to low
   
}

void display_driver_initialize(){
    
    
    TRISBbits.TRISB15 = 0;//set to output bit 15 for RS
    TRISDbits.TRISD5 = 0; //set as output (for RD5) of TRISD for R/W
    
    TRISEbits.TRISE0 = 0;//set as output
    TRISEbits.TRISE1 = 0;//set as output
    TRISEbits.TRISE2 = 0;//set as output
    TRISEbits.TRISE3 = 0;//set as output
    TRISEbits.TRISE4 = 0;//set as output
    TRISEbits.TRISE5 = 0;//set as output
    TRISEbits.TRISE6 = 0;//set as output
    TRISEbits.TRISE7 = 0;//set as output
   
    // Function set //  5 x 10 setup, dual line
    LATBbits.LATB15= 0;// RB15 is RS, set low 
    LATDbits.LATD5 = 0; // RD5 for R/W set low  
    
    LATEbits.LATE0 = 1;// dont care
    LATEbits.LATE1 = 1;// dont care
    LATEbits.LATE2 = 1; // 5x7 vs 5x10 dots
    LATEbits.LATE3 = 1; // 1 line vs 2 lines
    LATEbits.LATE4 = 1;
    LATEbits.LATE5 = 1;
    LATEbits.LATE6 = 0;
    LATEbits.LATE7 = 0; // set last four bits to 1 (DB0-3), and DB5-4 are also 1
    // Function set // 5 x 10 setup, dual line
    
    display_driver_enable(); // write to LCD controller
    
     // Turn on display 
    LATBbits.LATB15= 0;// RB15 is RS, set low   
    LATDbits.LATD5 = 0; // RD5 for R/W set low 
    
    LATEbits.LATE0 = 0; // blink on or off
    LATEbits.LATE1 = 0; // cursor on or off
    LATEbits.LATE2 = 1; // display on or off
    LATEbits.LATE3 = 1;
    LATEbits.LATE4 = 0;
    LATEbits.LATE5 = 0;
    LATEbits.LATE6 = 0;
    LATEbits.LATE7 = 0;
    //turn on display
    
    display_driver_enable(); // write to LCD controller
    display_driver_clear(); // call clear function
    
    int j;
    for(j=0; j<50000; j++){ // this is to wait 1.64 ms 
    };
    
    // Data entry mode
    LATBbits.LATB15= 0;// RB15 is RS, set low   
    LATDbits.LATD5 = 0; // RD5 for R/W set low 
    
    LATEbits.LATE0 = 0; // decrement mode
    LATEbits.LATE1 = 0; // entire shift off
    LATEbits.LATE2 = 1; //
    LATEbits.LATE3 = 0;
    LATEbits.LATE4 = 0;
    LATEbits.LATE5 = 0;
    LATEbits.LATE6 = 0;
    LATEbits.LATE7 = 0; // set last three bits to 1 (DB0-2, written 0000,0111)
    // Data entry mode
    display_driver_enable(); // write to LCD controller
}

        
void display_driver_clear(){
    
    LATBbits.LATB15= 0;// RB15 is RS, set low   
    LATDbits.LATD5 = 0; // RD5 for R/W set low 
    
    LATEbits.LATE0 = 1; // set only RD0 high
    LATEbits.LATE1 = 0; // 
    LATEbits.LATE2 = 0; // 
    LATEbits.LATE3 = 0;
    LATEbits.LATE4 = 0;
    LATEbits.LATE5 = 0;
    LATEbits.LATE6 = 0;
    LATEbits.LATE7 = 0;
    
    display_driver_enable(); // write to LCD controller
    
}

void display_driver_write(char* data, int length){
    
    int k;
    
     LATBbits.LATB15= 0;
    for(k=0; k<length; k++){// for for each char
        LATBbits.LATB15= 1;// RB15 is RS, set low   
        LATDbits.LATD5 = 0; // RD5 for R/W set low 
        LATE = data[k]; // write char ascii value to LATE
        display_driver_enable(); // write to LCD controller
        
    }
    
    
}

