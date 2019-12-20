/* 
 * File:   uart_interface.c
 * Author: Forest Davis-Hollander and Chunliang Tao
 *
 * Created on November 14, 2019
 * 
 * This program controls the UART communication on the PIC32. The uart_initialize function takes a given UART (1-6) and initializes its registers. The uart_write function waits until UART is ready before transmitting to an external device, also selecting between UART 1-6. The uart_read function waits for data to be available and then reads the data, also selecting between UART 1-6.
 */

#include "uart_interface.h"
#include <xc.h>


void uart_initialize(int uart_number){ // initialize uart
   
   
    if(uart_number == 1){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U1MODEbits.PDSEL = 0x0; //no parity
         U5MODEbits.STSEL = 0; // one stop
         U1STAbits.UTXEN = 1; // enable transmit
         U1STAbits.URXEN = 1; // enable receive
         U1MODEbits.BRGH = 0; // m =4
         // U1BRG = Fpb/(M * baud) - 1 
         U1BRG = 520; // 80 M/(16* 9600) - 1 = 520
         U1MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
   
    if(uart_number == 2){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U2MODEbits.PDSEL = 0x0; // no parity 
         U2MODEbits.STSEL = 0; // one stop bit
         U2STAbits.UTXEN = 1; // enable transmit
         U2STAbits.URXEN = 1; // enable receive
         U2MODEbits.BRGH = 0; // m =16
         // U1BRG = Fpb/(M * baud) - 1 

         U2BRG = 520; // 80 M/(16* 9600) - 1 = 521
         U2MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
    
    if(uart_number == 3){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U3MODEbits.PDSEL = 0x0; // no parity 
          U3MODEbits.STSEL = 0; // one stop bit
         U3STAbits.UTXEN = 1; // enable transmit
         U3STAbits.URXEN = 1; // enable receive
         U3MODEbits.BRGH = 0;
         // U1BRG = Fpb/(M * baud) - 1 

         U3BRG = 520; // 80 M/(16* 9600) - 1 = 521
         U3MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
    
    if(uart_number == 4){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U4MODEbits.PDSEL = 0x0; // no parity 
         U4MODEbits.STSEL = 0; // one stop bit
         U4STAbits.UTXEN = 1; // enable transmit
         U4STAbits.URXEN = 1; // enable receive
         U4MODEbits.BRGH = 0; // m =4
         // U1BRG = Fpb/(M * baud) - 1 

         U4BRG = 520; // 80 M/(16* 9600) - 1 = 521
         U4MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
    
    if(uart_number == 5){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U5MODEbits.PDSEL = 0x0; // no parity 
         U5MODEbits.STSEL = 0; // one stop bit
         U5STAbits.UTXEN = 1; // enable transmit
         U5STAbits.URXEN = 1; // enable receive
         U5MODEbits.BRGH = 0; // m =4
         // U1BRG = Fpb/(M * baud) - 1 

         U5BRG = 520; // 80 M/(16* 9600) - 1 = 521
         U5MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
    
    if(uart_number == 6){ /// do if for every UART
        // initialize UART1: 100 baud, odd parity, 1 stop bit
         U6MODEbits.PDSEL = 0x0; // no parity 
         U6MODEbits.STSEL = 0; // one stop bit
         U6STAbits.UTXEN = 1; // enable transmit
         U6STAbits.URXEN = 1; // enable receive
         U6MODEbits.BRGH = 0; // m =4
         // U1BRG = Fpb/(M * baud) - 1 

         U6BRG = 520; // 80 M/(16* 9600) - 1 = 521
         U6MODEbits.ON = 1; // turn on the uart
         // scope instructions: 10 ms/div, trigger on falling edge, single capture
    }
   

}

int uart_read(int uart_number){ // read from external device
   
    int data;
    while(!U2STAbits.URXDA) {;} // poll to see if there is data to read in RX FIFO
    data = U2RXREG; // data has arrived; read the byte
    return data; // return data
}

       

void uart_write(char data, int uart_number){ // write to external device
    
    
   if(uart_number == 1){ /// do if for every UART
       while(U1STAbits.UTXBF) { ; } // wait for UART to be ready
       U1TXREG = data; // write 
    }
   if(uart_number == 2){ /// do if for every UART
       while(U2STAbits.UTXBF) { ; } // wait for UART to be ready
       U2TXREG = data; // write 
    }
   if(uart_number == 3){ /// do if for every UART
       while(U3STAbits.UTXBF) { ; } // wait for UART to be ready
       U3TXREG = data; // write 
    }
   if(uart_number == 4){ /// do if for every UART
       while(U4STAbits.UTXBF) { ; } // wait for UART to be ready
       U4TXREG = data; // write 
    }
   if(uart_number == 5){ /// do if for every UART
       while(U5STAbits.UTXBF) { ; } // wait for UART to be ready
       U5TXREG = data; // write 
    }
   if(uart_number == 6){ /// do if for every UART
       while(U6STAbits.UTXBF) { ; } // wait for UART to be ready
       U6TXREG = data; // write 
    }
   
   
}