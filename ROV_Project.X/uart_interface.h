/* 
 * File:   uart_interface.h
 * Author: Chunliang Tao, Forest Davis-Hollander
 *
 * Created on November 13, 2019, 9:07 AM
 * This file initializes the functions that will be needed in the main program from the display_driver.c source file.
 */


void uart_initialize(int uart_number);
void uart_read(char * message, int maxLength);
void uart_write(char data, int uart_number);