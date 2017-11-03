// Author			: Fabian Kung
// Date				: 1 Feb 2016
// Filename			: Driver_USART_V100.h

#ifndef _DRIVER_USART_SAM4S_H
#define _DRIVER_USART_SAM4S_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//

#define __SCI_TXBUF2_LENGTH      8			// SCI transmit  buffer2 length in bytes.
#define __SCI_RXBUF2_LENGTH      8			// SCI receive  buffer2 length in bytes.		
		
// Data buffer and address pointers for wired serial communications.
extern uint8_t gbytTXbuffer2[__SCI_TXBUF2_LENGTH-1];
extern uint8_t gbytTXbufptr2;
extern uint8_t gbytTXbuflen2;
extern uint8_t gbytRXbuffer2[__SCI_RXBUF2_LENGTH-1];
extern uint8_t gbytRXbufptr2;

extern	SCI_STATUS gSCIstatus2;
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_USART_Driver(TASK_ATTRIBUTE *);

#endif
