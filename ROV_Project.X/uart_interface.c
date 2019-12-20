/* 
 * File:   main.c
 * Author: Forest Davis-Hollander
 *
 * Created on September 13, 2019, 9:07 AM
 * This program is the controls the subroutines that are needed in the main program. This contains the code for the display_driver_enable, display_driver_clear, display_driver_initialize, and display_driver_write subroutines. Each subroutine is in charge of their respective part of writing to the LCD.
 */


#include "uart_interface.h"
#include <xc.h>


void uart_initialize(int uart_number){
   
    if(uart_number == 2){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U2MODEbits.PDSEL = 0x0; // odd parity 
         U2STAbits.UTXEN = 1; // enable transmit
         U2STAbits.URXEN = 1; // enable receive
         U2MODEbits.BRGH = 0;
         // U1BRG = Fpb/(M * baud) - 1 

         U2BRG = 42; // 80 M/(16*115200) - 1 = 49,999
         U2MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
}

void uart_read(char * message, int maxLength){
   
    char data = 0;
    int complete = 0, num_bytes = 0;
    while(!complete){
        
        if(U2STAbits.URXDA){       
            data = U2RXREG;
            if((data == '\n') || (data == '\r')){
                complete = 1;
            }
            else{  
                message[num_bytes] = data;
                ++num_bytes;
                if(num_bytes >= maxLength){
                    num_bytes = 0;
                }
            }
           
        }
        
    }
    message[num_bytes] = '\0';
}

       

void uart_write(char data, int uart_number){

   if(uart_number == 1){ /// do if for every UART
       while(U1STAbits.UTXBF) { ; }
       U1TXREG = data; // write twice so we can see the stop bit
    }
   if(uart_number == 2){ /// do if for every UART
       while(U2STAbits.UTXBF) { ; }
       U2TXREG = data; // write twice so we can see the stop bit 
   }
   
   if(uart_number == 3){ /// do if for every UART
       while(U3STAbits.UTXBF) { ; }
       U3TXREG = data; // write twice so we can see the stop bit
    }
   if(uart_number == 4){ /// do if for every UART
       while(U4STAbits.UTXBF) { ; }
       U4TXREG = data; // write twice so we can see the stop bit
    }
   if(uart_number == 5){ /// do if for every UART
       while(U5STAbits.UTXBF) { ; }
       U5TXREG = data; // write twice so we can see the stop bit
    }
   if(uart_number == 6){ /// do if for every UART
       while(U6STAbits.UTXBF) { ; }
       U6TXREG = data; // write twice so we can see the stop bit
    }
   
   
}

