// Author			: Fabian Kung
// Date				: 29 March 2017
// Filename			: Driver_DACC_V100.h

#ifndef _DRIVER_DACC_SAM4S_H
#define _DRIVER_DACC_SAM4s_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//
				
extern unsigned int gunDAC_OutA;
extern unsigned int gunDAC_OutB;


//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_DACC_Driver(TASK_ATTRIBUTE *);

#endif
