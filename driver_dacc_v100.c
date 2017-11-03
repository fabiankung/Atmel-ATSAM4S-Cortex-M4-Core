//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2017, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File				: Drivers_UART_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 29 March 2017
// Toolsuites		: Atmel Studio 7.0 or later
//					  GCC C-Compiler
#include "osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.


//
// --- PUBLIC VARIABLES ---
//
unsigned int gunDAC_OutA;
unsigned int gunDAC_OutB;

//
// --- PRIVATE VARIABLES ---
//


//
// --- Process Level Constants Definition --- 
//



///
/// Process name	: Proce_DACC_Driver
///
/// Author			: Fabian Kung
///
/// Last modified	: 29 March 2017
///
/// Code version	: 1.00
///
/// Processor		: ARM Cortex-M4 family                   
///
/// Processor/System Resource 
/// PINS		: 1. Pin PB14 = DAC1, peripheral B, output.
///
/// MODULES		: 1. DACC (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global variable	: 
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_DACC_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_DACC_Driver: An RTOS is required with this function"
#endif

///
/// Description		: 
/// NOTE: 29 March 2017
/// This module temporary cannot be used as the max clock allowable is 50 MHz.  Here we are
/// using 120 MHz and the observation is the the DACC output the wrong value.  It cannot go
/// down to 0V output.

void Proce_DACC_Driver(TASK_ATTRIBUTE *ptrTask)
{

	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - Initialization.
				//PIOB->PIO_PDR = (PIOB->PIO_PDR) | PIO_PDR_P14;	// Set PB14 to be controlled by Peripheral.

				DACC->DACC_CHER |= DACC_CHER_CH1;		// Enable DAC channel 1.
				DACC->DACC_MR |= DACC_MR_ONE | DACC_MR_USER_SEL_CHANNEL1;	// Select DAC Channel 1.
				PMC->PMC_PCER0 |= PMC_PCER0_PID30;		// Enable peripheral clock to DACC (ID30)
				OSSetTaskContext(ptrTask, 1, 100);		// Next state = 1, timer = 100.
			break;
			
			case 1: // State 1 - See if DACC is ready to accept new data for conversion.
				if ((DACC->DACC_ISR & DACC_ISR_TXRDY) > 0)		// If DACC is ready to accept new conversion request.
				{
					DACC->DACC_CDR = 0x0000;
				}
				OSSetTaskContext(ptrTask, 2, 1); // Next state = 2, timer = 1.
			break;

			case 2: // State 2 - See if DACC is ready to accept new data for conversion.
			if ((DACC->DACC_ISR & DACC_ISR_TXRDY) > 0)		// If DACC is ready to accept new conversion request.
			{
				DACC->DACC_CDR = 0x02FF;
			}
			OSSetTaskContext(ptrTask, 3, 1); // Next state = 3, timer = 1.
			break;

			case 3: // State 3 - See if DACC is ready to accept new data for conversion.
			if ((DACC->DACC_ISR & DACC_ISR_TXRDY) > 0)		// If DACC is ready to accept new conversion request.
			{
				DACC->DACC_CDR = 0x0FFF;
			}
			OSSetTaskContext(ptrTask, 1, 1); // Next state = 2, timer = 1.
			break;

			default:
				OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
			break;
		}
	}
}


