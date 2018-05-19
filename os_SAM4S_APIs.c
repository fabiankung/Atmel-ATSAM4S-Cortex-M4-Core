///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  APPLICATION PROGRAM INTERFACE ROUTINES FOR SAM4S ARM CORTEX-M4 MICROCONTROLLER
//
//  (c) Copyright 2015-2018, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Filename			: os_SAM4S_APIs.c
// Author			: Fabian Kung
// Last modified	: 18 May 2018
// Version			: 1.02
// Description		: This file contains the implementation of all the important routines
//                    used by the OS and the user routines. Most of the routines deal with
//                    micro-controller specifics resources, thus the functions have to be
//                    rewritten for different micro-controller family. Only the function
//                    prototype (call convention) of the routine is to be maintained.  In this
//                    way the OS can be ported to different micro-controller family.
// Toolsuites		: AtmelStudio 7.0 or above
//                	  GCC C-Compiler 
// Micro-controller	: Atmel SAM4S ARM Cortex-M4 families.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "osmain.h"

// --- GLOBAL AND EXTERNAL VARIABLES DECLARATION ---


// --- FUNCTIONS' PROTOTYPES ---


// --- FUNCTIONS' BODY ---

//////////////////////////////////////////////////////////////////////////////////////////////
//  BEGINNING OF CODES SPECIFIC TO SAM4SD16 MICROCONTROLLER	    //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


// Function name	: ClearWatchDog 
// Author			: Fabian Kung
// Last modified	: 20 Nov 2015
// Purpose			: Reset the Watch Dog Timer
// Arguments		: None
// Return			: None
// Description		: On the ARM Cortex-M micro-controller, the Watchdog Timer is enabled by default
//                    after power on reset.  The timeout period is determined by the 12-bits field 
//                    WDV of the WDT_MR register. The default value is 0xFFF.  The Watchdog Timer is
//                    driven by internal slow clock of 32.768 kHz with pre-scaler of 128, which work
//                    out to 16 seconds timeout period.
inline void ClearWatchDog(void)
{
	WDT->WDT_CR = (WDT->WDT_CR) | WDT_CR_WDRSTT | WDT_CR_KEY_PASSWD;	// Reload the Watchdog Timer.
																		// Note that to prevent accidental
																		// write, this register requires a Key or password.
}

/// Function Name	: SAM4S_Init
/// Author			: Fabian Kung
/// Last modified	: 18 May 2018
/// Description		: This function performs further initialization of the Cortex-M
///                   processor, namely:
///					  1. Setup processor main oscillator and clock generator circuit.
///                   2. Setup processor flash memory controller wait states.
///                   3. Setup the SysTick system timer.
///                   4. Enable the cache controller.
///					  5. Also initialized the micro-controller peripherals and
///                      I/O ports to a known state. For I/O ports, all pins will
///                      be set to
///                     (a) Assign to PIO module (PIOA to PIOC)
///					    (b) Digital mode,
///                     (c) Output and
///                     (d) A logic '0'.
/// Arguments		: None
/// Return			: None
/// 
void SAM4S_Init()
{
	// Upon reset, the fast RC oscillator is enabled with 4 MHz frequency selected as the source of MAINCK.
	// Routines to enable and start-up the main crystal oscillator via the Power Management Controller (PMC)
	PMC->CKGR_MOR = (PMC->CKGR_MOR) | CKGR_MOR_MOSCXTST(100) | CKGR_MOR_KEY_PASSWD;	// Main crystal oscillator start-up time,
	// 100x8=800 slow clock cycles.  Slow clock runs
	// at 32 kHz.  Note that to prevent accidental
	// write, this register requires a Key or password.
	PMC->CKGR_MOR = (PMC->CKGR_MOR) | CKGR_MOR_MOSCXTEN | CKGR_MOR_KEY_PASSWD;		// Enable main 8 MHz crystal oscillator.
	while ((PMC->PMC_SR & PMC_SR_MOSCXTS) == 0) {}									// Wait until the main crystal oscillator is stabilized.
	PMC->CKGR_MOR = (PMC->CKGR_MOR) | CKGR_MOR_MOSCSEL | CKGR_MOR_KEY_PASSWD;			// Select the main crystal oscillator.
	while ((PMC->PMC_SR & PMC_SR_MOSCSELS) == 0) {}									// Wait until main oscillator selection is done.
	PMC->CKGR_MOR = ((PMC->CKGR_MOR) & ~CKGR_MOR_MOSCRCEN) | CKGR_MOR_KEY_PASSWD;		// Disable the on-chip fast RC oscillator.

	
	
	// Set FWS (Flash memory wait state) according to clock configuration
	// This has to be set first before we change the main clock (MCK) of the core to higher frequency
	// Please refer to the device datasheet on Enhanced Embedded Flash Controller (EEFC) on the wait state to insert depending on
	// core clock frequency
	// For fcore = 120 MHz, FWS = 5, e.g. 6 wait states.
	// For fcore = 4 MHz, FWS = 0, e.g. 1 wait state.
	// For fcore = 8-20 MHz, FWS = 1, e.g. 2 wait states.
	EFC0->EEFC_FMR = EEFC_FMR_FWS(5);
	#if defined(ID_EFC1)
	EFC1->EEFC_FMR = EEFC_FMR_FWS(5);
	#endif

	// Routines to enable PLLB and use this as main clock via the Power Management Controller (PMC)
	PMC->CKGR_PLLBR = (PMC->CKGR_PLLBR & ~CKGR_PLLBR_PLLBCOUNT_Msk) | CKGR_PLLBR_PLLBCOUNT(100) | CKGR_PLLBR_DIVB(0) | CKGR_PLLBR_MULB(0);	// Disable PLLB first.
	PMC->CKGR_PLLBR = (PMC->CKGR_PLLBR & ~CKGR_PLLBR_PLLBCOUNT_Msk) | CKGR_PLLBR_PLLBCOUNT(100) | CKGR_PLLBR_DIVB(2) | CKGR_PLLBR_MULB(30);	// Enable PLLB.
	// Here fxtal (crystal oscillator) = 8 MHz
	// Thus fin = fxtal / DIVB = 8/2 = 4 MHz
	// fPLLB = fin x MULB = 4 * 30 = 120 MHz.
	// fcore = fPLLB = 120 MHz.
	while ((PMC->PMC_SR & PMC_SR_LOCKB) == 0) {}				// Wait until PLLB is locked.
	
	// fcore = fPLLB / pre-scaler = fPLLB / 1 = fPLLB = 120 MHz.
	// Note: we can also set fPLLB to 240 MHz and set pre-scalar to 2 as follows:
	//PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | PMC_MCKR_PRES_CLK_2;		// Set pre-scalar to divide-by-2.
	//while ((PMC->PMC_SR & PMC_SR_MCKRDY) == 0) {}				// Wait until Master Clock is ready.

	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_PLLB_CLK; 		// Change master clock source to PLLB.
	while ((PMC->PMC_SR & PMC_SR_MCKRDY) == 0) {}				// Wait until Master Clock is ready.

	// Disable all clock signals to non-critical peripherals as default (to save power).
	// Note: Peripherals 0-7 are system critical peripheral such as Supply Controller, Reset Controller, Real-Time Clock/Timer, Watchdog Timer,
	// Power Management Controller and Flash Controller.  The clock to these peripherals cannot be disabled.
	PMC->PMC_PCDR0 = 0xFFFFFF00;		// Disable clock to peripheral ID8 to ID31.
	PMC->PMC_PCDR1 = 0x0000000F;		// Disable clock to peripheral ID32 to ID34.

	
	// Setup Port A and Port B IO ports.
	// --- Setup PIOA ---
	PMC->PMC_PCER0 |= PMC_PCER0_PID11;		// Enable peripheral clock to PIOA (ID11).
	PMC->PMC_PCER0 |= PMC_PCER0_PID12;		// Enable peripheral clock to PIOB (ID12).
			
	// Setup pin PA17 as output.  General purpose output.
	//PIOA->PIO_PER |= PIO_PER_P17;	// PA17 is controlled by PIO.
	//PIOA->PIO_OER |= PIO_OER_P17;	// Enable output write to PA17.
	//PIOA->PIO_OWER |= PIO_OWER_P17;	// Set Output Write Status Register bit (if we are using ODSR to change the value of PA17)
	
	// Setup pin PA24 as input. General purpose input.
	//PIOA->PIO_PPDDR |= PIO_PPDDR_P24;  // Disable internal pull-down to PA24.
	//PIOA->PIO_PUER |= PIO_PUER_P24; // Enable internal pull-up to PA24.
	//PIOA->PIO_PER |= PIO_PER_P24;	// PA24 is controlled by PIO.
	//PIOA->PIO_ODR |= PIO_ODR_P24;	// Disable output write to PA24.
	//PIOA->PIO_IFER |= PIO_IFER_P24;	// Enable input glitch filter to PA24. This is optional.
	
	PIOA->PIO_PER = 0xFFFFFFFF;		// All PA1-PA32 are controlled by PIO.
	//PIOA->PIO_OER = 0xFFFFFFFF;		// Set PIOA to outputs.
	//PIOA->PIO_OWER = 0xFFFFFFFF;	// Enable output write to PIOA.
									// Set Output Write Status Register bit (if we are using ODSR to change the value of PA1-PA32)
	PIOA->PIO_OER = 0x7E7FFF;		// Set PIOA to outputs.
	PIOA->PIO_OWER = 0x7E7FFF;		// Enable output write to PIOA.
									
	PIOB->PIO_PER = 0xFFFFFFFF;		// All PB1-PB32 are controlled by PIO.
	PIOB->PIO_OER = 0xFFFFFFFF;		// Set PIOB to outputs.
	PIOB->PIO_OWER = 0xFFFFFFFF;	// Enable output write to PIOB.
									// Set Output Write Status Register bit (if we are using ODSR to change the value of PB1-PB32)									
	
	// Note: 16 Oct 2015, the following is not needed, by default SysTick is being triggered with processor clock.
	// The SysTick module is triggered from the output of the Master Clock (MCK) divided by 8.  Since MCK = fCore,
	// the timeout for SysTick = [SysTick Value] x 8 x (1/fCore).
	// For fCore = 120 MHz, SysTick Value = 100
	//SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;	// Set this flag, indicate clock source for SysTick from the processor clock.
	// End of note.
	//SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;	// Enable SysTick exception request when count down to zero.
	SysTick->LOAD = __SYSTICKCOUNT;	// Set reload value.
	SysTick->VAL = __SYSTICKCOUNT;	// Reset current SysTick value.
	SysTick->CTRL = SysTick->CTRL & ~(SysTick_CTRL_COUNTFLAG_Msk);	// Clear Count Flag.
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	// Enable SysTick.
	
	// Enable the Cortex-M Cache Controller
	if (((CMCC->CMCC_SR) & CMCC_SR_CSTS) == 0)	// Check the CSTS value, if 0 start the Cache Controller.
	{
		CMCC->CMCC_CTRL |= CMCC_CTRL_CEN; // Enable the Cache Controller.
	}

}

// Function name	: OSEnterCritical
// Author			: Fabian Kung
// Last modified	: 24 April 2007
// Description		: Disable all processor interrupts for important tasks
//					  involving Stacks, Program Counter other critical
//	                  processor registers.
void OSEnterCritical(void)
{

}											

// Function name	: OSExitCritical
// Author		: Fabian Kung
// Last modified	: 24 April 2007
// Description		: Enable all processor interrupts for important tasks.
void OSExitCritical(void)
{
 
}											


// Function name	: OSProce1
// Author			: Fabian Kung
// Last modified	: 20 Nov 2015
// Description		: Blink an indicator LED1 to show that the micro-controller is 'alive'.
#define _LED1_ON_US	500000		// LED1 on period in usec, i.e. 500msec.

void OSProce1(TASK_ATTRIBUTE *ptrTask)
{
    switch (ptrTask->nState)
    {
	case 0: // State 0 - On Indicator LED1
            PIN_OSPROCE1_SET;													// Set PA17.                                                
            OSSetTaskContext(ptrTask, 1, _LED1_ON_US/__SYSTEMTICK_US);          // Next state = 1, timer = _LED_ON_US.
	break;

	case 1: // State 1 - Off Indicator LED1
            PIN_OSPROCE1_CLEAR;													// Clear PA17.                                                  
            OSSetTaskContext(ptrTask, 0, _LED1_ON_US/__SYSTEMTICK_US);          // Next state = 0, timer = 5000.
            break;

        default:
            OSSetTaskContext(ptrTask, 0, 0);                                    // Back to state = 0, timer = 0.
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//  END OF CODES SPECIFIC TO SAM4SD16 MICROCONTROLLER	   ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
