/* 
 * File:   main.c
 * Author: Forest Davis-Hollander
 *
 * Created on September 13, 2019, 9:07 AM
 * This file initializes the functions that will be needed in the main program from the display_driver.c source file.
 */


#ifndef DISPLAY_DRIVER_H
#define	DISPLAY_DRIVER_H


void display_driver_enable(); //initialize function

void display_driver_initialize(); //initialize function

void display_driver_clear(); //initialize function

void display_driver_write(char* data, int length); //initialize function

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* DISPLAY_DRIVER_H */

