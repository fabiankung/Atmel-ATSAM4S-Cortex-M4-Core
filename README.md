# Atmel-ATSAM4S-Cortex-M4-Core
This project describes the core circuit for Atmel ATSAM4SD16B ARM Cortex M4 micro-controller. Please see this [blog](https://fkeng.blogspot.com/2016/02/atmel-arm-cortex-m4-microcontroller.html) for further details. The particular micro-controller I am using is in TQFP-64 package.  
The basic circuit contains the voltage regulator, decoupling capacitors, reset circuit, 8 MHz crystal oscillator and SWD (a derivative of JTAG) programming port.
The files located in the github site contains the schematic which implement the four elements above and also an explanation of the connection from the programmer to the program/debug port of the micro-controller.  
Here I am using Atmel ICE for programming the chip.
Together with this repository I include driver routines in C to use the micro-controller peripherals.  These drivers use state-machine implementation and can be incorporated in a real-time operating system or a simple scheduler.
