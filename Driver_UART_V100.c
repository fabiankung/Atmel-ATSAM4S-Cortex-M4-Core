//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2015, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File				: Drivers_UART_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 28 Dec 2015
// Toolsuites		: Atmel Studio 6.2 or later
//					  GCC C-Compiler
#include "osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.


//
// --- PUBLIC VARIABLES ---
//
// Data buffer and address pointers for wired serial communications (UART).
uint8_t gbytTXbuffer[__SCI_TXBUF_LENGTH-1];       // Transmit buffer.
uint8_t gbytTXbufptr;                             // Transmit buffer pointer.
uint8_t gbytTXbuflen;                             // Transmit buffer length.
uint8_t gbytRXbuffer[__SCI_RXBUF_LENGTH-1];       // Receive buffer length.
uint8_t gbytRXbufptr;                             // Receive buffer length pointer.

//
// --- PRIVATE VARIABLES ---
//


//
// --- Process Level Constants Definition --- 
//

//#define	_UART_BAUDRATE_kBPS	9.6	// Default datarate in kilobits-per-second, for HC-05 module.
//#define	_UART_BAUDRATE_kBPS 38.4	// Default datarate in kilobits-per-second for HC-05 module in AT mode.
#define	_UART_BAUDRATE_kBPS 115.2	// Default datarate in kilobits-per-second
//#define	_UART_BAUDRATE_kBPS 128.0	// Default datarate in kilobits-per-second
//#define	_UART_BAUDRATE_kBPS 230.4	// Default datarate in kilobits-per-second


///
/// Process name	: Proce_UART_Driver
///
/// Author			: Fabian Kung
///
/// Last modified	: 28 Dec 2015
///
/// Code version	: 1.00
///
/// Processor		: ARM Cortex-M4 family                   
///
/// Processor/System Resource 
/// PINS		: 1. Pin PA9 = URXD0, peripheral A, input.
///  			  2. Pin PA10 = UTXD0, peripheral A, output.
///               3. PIN_ILED2 = indicator LED2.
///
/// MODULES		: 1. UART0 (Internal) and PDC (Internal).
///               2. PDC (Peripheral DMA Controller) (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global variable	: gbytRXbuffer[]
///                   gbytRXbufptr
///                   gbytTXbuffer[]
///                   gbytTXbufptr
///                   gbytTXbuflen
///                   gSCIstatus
///

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_UART_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_UART_Driver: An RTOS is required with this function"
#endif

///
/// Description		: 1. Driver for built-in UART0 Module.
///                   2. Serial Communication Interface (UART) transmit buffer manager.
///                      Data will be taken from the SCI transmit buffer gbytTXbuffer in FIFO basis
///                      and transmitted via USART module.  Maximum data length is determined by the
///						 constant _SCI_TXBUF_LENGTH in file "osmain.h".
///                      Data transmission can be done with or without the assistance of the Peripheral
///                      DMA Controller (PDC).
///                   3. Serial Communication Interface (UART) receive buffer manager.
///					Data received from the USART module of the micro-controller will be
///					transferred from the USART registers to the RAM of the micro-controller
///					called SCI receive buffer (gbytRXbuffer[]).
///					The flag bRXRDY will be set to indicate to the user modules that valid
///					data is present.
///					Maximum data length is determined by the constant _SCI_RXBUF_LENGTH in
///					file "osmain.h".
///
///
/// Example of usage : The codes example below illustrates how to send 2 bytes of character,
///			'a' and 'b' via UART without PDC assistance.
///          if (gSCIstatus.bTXRDY == 0)	// Check if any data to send via UART.
///          {
///             gbytTXbuffer[0] = 'a';	// Load data.
///		   	    gbytTXbuffer[1] = 'b';
///		   	    gbytTXbuflen = 2;		// Set TX frame length.
///		  	    gSCIstatus.bTXRDY = 1;	// Initiate TX.
///          }
///
/// Example of usage : The codes example below illustrates how to send 100 bytes of character,
///			 via UART with PDC assistance.  It is assumed the required data has been stored
///          in the transmit buffer gbytTXbuffer[0] to gbytTXbuffer[99] already.
///					PDC_UART0->PERIPH_TPR = gbytTXbuffer;	// Setup DMA for UART0 transmit operation.
///					PDC_UART0->PERIPH_TCR = 100;
///					PDC_UART0->PERIPH_TNPR = gbytTXbuffer;
///					PDC_UART0->PERIPH_TNCR = 0;
///					PDC_UART0->PERIPH_PTCR = PDC_UART0->PERIPH_PTCR | PERIPH_PTCR_TXTEN;	// Enable transmitter transfer.
///					gSCIstatus.bTXDMAEN = 1;	// Indicate UART transmit with DMA.
///					gSCIstatus.bTXRDY = 1;		// Initiate TX.
///					PIN_LED2_SET;				// Lights up indicator LED2.
///
/// Example of usage : The codes example below illustrates how to retrieve 1 byte of data from
///                    the UART receive buffer.
///			if (gSCIstatus.bRXRDY == 1)	// Check if UART receive any data.
///		    {
///             if (gSCIstatus.bRXOVF == 0) // Make sure no overflow error.
///			    {
///					bytData = gbytRXbuffer[0];	// Get 1 byte and ignore all others.
///             }
///             else
///             {
///					gSCIstatus.bRXOVF = 0; 	// Reset overflow error flag.
///             }
///             gSCIstatus.bRXRDY = 0;	// Reset valid data flag.
///             gbytRXbufptr = 0; 		// Reset pointer.
///			}
///
/// Note: Another way to check for received data is to monitor the received buffer pointer
/// gbytRXbufptr.  If no data this pointer is 0, a value greater than zero indicates the
/// number of bytes contain in the receive buffer.


void Proce_UART_Driver(TASK_ATTRIBUTE *ptrTask)
{

	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - UART0 Initialization.
				// Setup IO pins mode and configure the peripheral pins:

				// Setup pin PA9 as input. General purpose input.
				PIOA->PIO_PPDDR |= PIO_PPDDR_P9;  // Disable internal pull-down to PA9.
				PIOA->PIO_PUER |= PIO_PUER_P9;	// Enable internal pull-up to PA9.
				//PIOA->PIO_PER |= PIO_PER_P9;	// PA9 is controlled by PIO.
				PIOA->PIO_ODR |= PIO_ODR_P9;	// Disable output write to PA9.
				//PIOA->PIO_IFER |= PIO_IFER_P9;	// Enable input glitch filter to PA9. This is optional.
 				
				 // 24 Nov 2015: To enable a peripheral, we need to:
 				// 1. Assign the IO pins to the peripheral.
 				// 2. Select the correct peripheral block (A, B, C or D).
 				PIOA->PIO_PDR = (PIOA->PIO_PDR) | PIO_PDR_P9;	// Set PA9 and PA10 to be controlled by Peripheral.
 				PIOA->PIO_PDR = (PIOA->PIO_PDR) | PIO_PDR_P10;	// UART0 resides in Peripheral block A, with 
																// PA9 = URXD0 and PA10 = UTXD0.
 																
 				PIOA->PIO_ABCDSR[0] = (PIOA->PIO_ABCDSR[0]) & ~PIO_ABCDSR_P9;	// Select peripheral block A for
 				PIOA->PIO_ABCDSR[1] = (PIOA->PIO_ABCDSR[1]) & ~PIO_ABCDSR_P9;	// PA9.
 				PIOA->PIO_ABCDSR[0] = (PIOA->PIO_ABCDSR[0]) & ~PIO_ABCDSR_P10;	// Select peripheral block A for
 				PIOA->PIO_ABCDSR[1] = (PIOA->PIO_ABCDSR[1]) & ~PIO_ABCDSR_P10;	// PA10.

				// Setup baud rate generator register.  
				// Baudrate = (Peripheral clock)/(16xCD)
				// for CD = 65, baud rate = 115.38 kbps
				// for CD = 781, baud rate = 9.60 kbps
				// for CD = 32, baud rate = 234.375 kbps
                if (_UART_BAUDRATE_kBPS == 230.4)
				{
					UART0->UART_BRGR = 33;						// Approximate 230.4 kbps
				}
				else
				{
					UART0->UART_BRGR = (__FPERIPHERAL_MHz*1000)/(16*_UART_BAUDRATE_kBPS);
				}
				//UART0->UART_BRGR = 17;					// Approximate 460.8 kbps
				                
				// Setup USART0 operation mode part 1:
				// 1. Enable UART0 RX and TX modules.
				// 2. Channel mode = Normal.
				// 3. 8 bits data, no parity, 1 stop bit.
				// 4. No interrupt.
				
				UART0->UART_MR = UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL;	// Set parity and channel mode.
				UART0->UART_CR = UART0->UART_CR | UART_CR_TXEN | UART_CR_RXEN; // Enable both transmitter and receiver.
				                                
				gbytTXbuflen = 0;               // Initialize all relevant variables and flags.
				gbytTXbufptr = 0;
				gSCIstatus.bRXRDY = 0;	
				gSCIstatus.bTXRDY = 0;
				gSCIstatus.bRXOVF = 0;
                                
				gbytRXbufptr = 0;
                PIN_LED2_CLEAR;							// Off indicator LED2.
				PMC->PMC_PCER0 |= PMC_PCER0_PID8;		// Enable peripheral clock to UART0 (ID8)
				OSSetTaskContext(ptrTask, 1, 100);		// Next state = 1, timer = 100.
			break;
			
			case 1: // State 1 - Transmit and receive buffer manager.
				// Check for data to send via UART.
				// Note that the transmit buffer is only 2-level deep in ARM Cortex-M4 micro-controllers.
				if (gSCIstatus.bTXRDY == 1)                         // Check if valid data in SCI buffer.
				{
					if (gSCIstatus.bTXDMAEN == 0)					// Transmit without DMA.
					{
						while ((UART0->UART_SR & UART_SR_TXRDY) > 0)// Check if UART transmit holding buffer is not full.
						{
							PIN_LED2_SET;							// On indicator LED2.
							if (gbytTXbufptr < gbytTXbuflen)		// Make sure we haven't reach end of valid data to transmit. 
							{
								UART0->UART_THR = gbytTXbuffer[gbytTXbufptr];	// Load 1 byte data to UART transmit holding buffer.
								gbytTXbufptr++;                     // Pointer to next byte in TX buffer.
							}
							else                                    // End of data to transmit.
							{
								gbytTXbufptr = 0;                   // Reset TX buffer pointer.
								gbytTXbuflen = 0;                   // Reset TX buffer length.
								gSCIstatus.bTXRDY = 0;              // Reset transmit flag.
								PIN_LED2_CLEAR;                     // Off indicator LED2.
								break;
							}
						}
                    }
					else											// Transmit with DMA.
					{
						if ((UART0->UART_SR & UART_SR_ENDTX) > 0)	// Check if DMA UART transmit is completed.
						{
							gSCIstatus.bTXRDY = 0;					// Reset transmit flag.
							PIN_LED2_CLEAR;							// Off indicator LED2.
							break;							
						}
					}
				}


				// Check for data to receive via UART.
				// Note that the receive FIFO buffer is only 2-level deep in ARM Cortex-M4 micro-controllers.
                // Here we ignore Parity error.  If overflow or framing error is detected, we need to write a 1 
				// to the bit RSTSTA to clear the error flags.  It is also advisable to reset the receiver.
                                
				if (((UART0->UART_SR & UART_SR_FRAME) == 0) && ((UART0->UART_SR & UART_SR_OVRE) == 0)) 
                {															// Make sure no hardware overflow and 
																			// and framing error.
                    while ((UART0->UART_SR & UART_SR_RXRDY) > 0)			// Check for availability of data received.
																			// available at UART
                    {
                        PIN_LED2_SET;										// On indicator LED2.
                        if (gbytRXbufptr < __SCI_RXBUF_LENGTH)				// check for data overflow.
                        {													// Read a character from USART.
                            gbytRXbuffer[gbytRXbufptr] = UART0->UART_RHR;	// Get received data byte.
                            gbytRXbufptr++;									// Pointer to next byte in RX buffer.
                            gSCIstatus.bRXRDY = 1;							// Set valid data flag.
                        }
                        else 												// data overflow.
                        {
                            gbytRXbufptr = 0;								// Reset buffer pointer.
                            gSCIstatus.bRXOVF = 1;							// Set receive data overflow flag.
                        }
                        
                    }
                }
                else														// Hard overflow or/and framing error.
                {
                    UART0->UART_CR = UART0->UART_CR | UART_CR_RSTSTA;		// Clear overrun and framing error flags.
					//UART0->UART_CR = UART0->UART_CR | UART_CR_RSTRX;		// Reset the receiver.		
                    gbytRXbufptr = 0;										// Reset buffer pointer.
                    gSCIstatus.bRXOVF = 1;									// Set receive data overflow flag.
                } 
				
				OSSetTaskContext(ptrTask, 1, 1); // Next state = 1, timer = 1.
			break;

			default:
				OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
			break;
		}
	}
}


