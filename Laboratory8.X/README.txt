Name: Forest Davis-Hollander, Chunliang Tao
Email:fdavisho@u.rochester.edu, ctao4@u.rochester.edu
Date: 20 November 2019

We affirm that we have not given or received any unauthorized help on this assignment, and that this work is our own.

This program uses UART2 to communicate with a servo motor controller. The program first initializes the desired UART2 and then continuously loops until a change notification is detected. Depending on which button was pressed, the servo motor will rotate to either the home position, 45 degrees, or -45 degrees.

Note that we used Pin 50 for our transmission to the servo motor controller (per the lab setup) and kept the stnadard configuration to the remaining servor motor controller wires per Professor Howard's setup in the lab session. We also wired Pin 11 to Pin 92 for button S5 to CN9 so we could use button S5.

Additional notes: Go easy on the push buttons, allow them a second to settle before pressing them again.
