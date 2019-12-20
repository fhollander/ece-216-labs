/* 
 * File:   uart_interface.h
 * Author: Chunliang Tao, Forest Davis-Hollander
 *
 * Created on November 13, 2019, 9:07 AM
 * This file initializes the functions that will be needed in the main program for reading, writing, and initializing UART.
 */


void uart_initialize(int uart_number); // initialize function for uart initialize
int uart_read(int uart_number); // initialize function for uart read
void uart_write(char data, int uart_number); // initialize uart function for write