/*
 * ATSAM4SD16B.c
 *
 * Created: 2/1/2016 11:05:57 AM
 *  Author: Fabian Kung
 */ 

#include "osmain.h"

// --- Include file for libraries ---
#include "./C_Library/Driver_I2C_V100.h"
#include "./C_Library/Driver_UART_V100.h" 
#include "./C_Library/Driver_TCM8230.h" 
#include "./C_Library/Driver_USART_V100.h"  
//#include "./C_Library/Driver_HC_05_V100.h"
//#include "./C_Library/Driver_DACC_V100.h"

#include "User_Task.h" 

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	int ni = 0;

	/* Initialize the SAM system */
	SystemInit();				// CMSIS initialization.
	SAM4S_Init();				// Custom initialization.
	OSInit();                   // Custom initialization: Initialize the RTOS.
	gnTaskCount = 0; 			// Initialize task counter.

	// Initialize core OS processes.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], OSProce1);					// Start blinking LED process.

	// Initialize library processes.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_I2C0_Driver);		// I2C0 driver.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_UART_Driver);		// UART0 driver.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_USART_Driver);		// USART0 driver.
	// Initialize user processes.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_TCM8230_Driver);		// CMOS camera driver.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_MessageLoop_StreamImage);	// User task 1, stream image to external display.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_Camera_LED_Driver);	// Head/camera LED driver.
	//OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_EyeLED_Effect);		
	
	
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_Image1);				// User task 3, process image captured.
	OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_Image2);				// User task 4, process image captured.
	//OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_Image3);				// User task 5, process image captured.
	//OSCreateTask(&gstrcTaskContext[gnTaskCount], Proce_Image4);				// User task 6, process image captured.

	while (1)
	{
		// --- Check SysTick until time is up, then update each process's timer ---
		if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) > 0)		// Check if SysTick counts to 0 since the last read.
		{
			//PIOA->PIO_ODSR |= PIO_ODSR_P19;
			PIOB->PIO_ODSR |= PIO_ODSR_P1;			// Set PB1
			OSEnterCritical();

			if (gnRunTask == 1)						// If task overflow occur trap the controller
			{										// indefinitely and turn on indicator LED1.
				while (1)
				{
					ClearWatchDog();				// Clear the Watch Dog Timer.
					PIN_OSPROCE1_SET; 				// Turn on indicator LED1.
				}
			}

			gnRunTask = 1;							// Assert gnRunTask.
			gunClockTick++; 						// Increment RTOS clock tick counter.
			for (ni = 0; ni < gnTaskCount; ni++)		// Using for-loop produce more efficient
				// assembly codes.
			{
				if (gstrcTaskContext[ni].nTimer > 0) // Only decrement timer if it is greater than zero.
				{
					--(gstrcTaskContext[ni].nTimer); // Decrement timer for each process.
				}
			}

			OSExitCritical();
			//PIOA->PIO_ODSR &= ~PIO_ODSR_P19;	
			PIOB->PIO_ODSR &= ~PIO_ODSR_P1;			// Clear PB1
		}

		// --- Run processes ---
		ClearWatchDog();		// Clear the Watch Dog Timer.
		if (gnRunTask > 0) 		// Only execute tasks/processes when gnRunTask is not 0.
		{
			for (ni = 0; ni < gnTaskCount; ni++)
			{
				// Only execute a process/task if it's timer = 0.
				if (gstrcTaskContext[ni].nTimer == 0)
				{
					// Execute user task by dereferencing the function pointer.
					(*((TASK_POINTER)gfptrTask[ni]))(&gstrcTaskContext[ni]);
				}
			}
			gnRunTask = 0; 		// Reset gnRunTask.        
		}
	}
}
