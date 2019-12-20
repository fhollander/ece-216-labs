Forest Davis-Hollander, Chunliang Tao
fdavisho@u.rochester.edu, ctao4@u.rochester.edu

6 November 2019

We affirm that we have not given or received any unauthorized help on this assignment, and that this work is our own.

This program reads from a 48 count encoder using Core Timer, so encoder is read at a fixed rate. We designed a PI controller to change the duty cycle of the PWM signal based on the desired reference angle, allowing the motor to stop within 10 degrees of the reference angle. By using change notification, each push button press would increase/decrease the reference angle by either 45 or 90 degrees. Also, we designed a logic, that whenever there is overshoot, it reverses rotation direction and aims for reference to guarantee it has very small variance. Besides that, since every time a push button being pressed and released counted as twice the signal change, we designed a "delay", during which time if someone push the button for more than 1/3 second and release, it still counts only once instead of twice. However, to guarantee the project working perfectly, please don't press and hold the button for too long.

Note that we used Pin 72 for our signal from OC1 which was sent to the PWM / D2 channel for motor M1. Pin 12 was wired to yellow wire on encoder, and Pin 14 wired to white wire on encoder. We followed the standard configuration for the rest of the setup as described in the lab manual, with the exception of connecting M1IN1 to P25 and connecting M1IN2 to P24 for direction control. Finally, we connected P11 to P92 for CN interrupt for push button S5.