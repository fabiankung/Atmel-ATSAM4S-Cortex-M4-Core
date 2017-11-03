//
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File				: Drivers_TCM8230.c
// Author(s)		: Fabian Kung
// Last modified	: 13 April 2017
// Tool-suites		: Atmel Studio 7.0 or later
//                    GCC C-Compiler

#include "osmain.h"
#include "Driver_I2C_V100.h"
#include "Driver_UART_V100.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.

	
//
//
// --- PUBLIC VARIABLES ---
//
#define _HRESOLUTION_QQVGA   160
#define _VRESOLUTION_QQVGA   120
#define _NOPIXELSINFRAME	 19200

int16_t gn16Pixel[160]; // Column					
int		gnFrameCounter = 0;
int		gnImageWidth = _HRESOLUTION_QQVGA;
int		gnImageHeight = _VRESOLUTION_QQVGA;
int		gnCameraLED = 0;
int		gnEyeLEDRight = 0;			// Visible light LED for the eyes control registers.
int		gnEyeLEDLeft = 0;

unsigned int gunImgAtt[160][120];   // Image attribute, 160 columns and 120 rows.
// bit6 - bit0 = Luminance information, 7-bits.
// bit7 - Marker flag, set to 1 if we wish the external/remote display to highlight
//        this pixel.
// bit16 - bit8 = Hue information, 9-bits, 0 to 360, No hue (gray scale) = 511.
// bit22 - bit17 = Saturation, 6-bits, 0-63 (63 = pure spectral).
// bit30 - bit23 = Luminance gradient, 8-bits.
// bit31 - Special flag.  

#define     _LUMINANCE_MASK     0x0000007F  // luminance mask, bit6-0.
#define     _CLUMINANCE_MASK    0xFFFFFF00  // One's complement of luminance mask.
#define     _LUMINANCE_SHIFT    0
#define     _HUE_MASK           0x0001FF00  // bit16-8
#define     _HUE_SHIFT          8
#define     _SAT_MASK           0x003E0000  // Saturation mask, bit22-17.
#define     _CSAT_MASK          0xFFC1FFFF  // One's complement of saturation mask.
#define     _SAT_SHIFT          17
#define     _NO_HUE             0x1FF       // Value for no hue, in decimal 511.
// Valid hue ranges from 0 to 360.
#define     _GRAD_MASK          0x7F800000  // bit30-23
#define     _CGRAD_MASK         0x807FFFFF  // One's complement of gradient mask.
#define     _GRAD_SHIFT         23
#define     _MAX_GRADIENT       255

unsigned int	gunR;						// Test variables.
unsigned int	gunG;
unsigned int	gunB;
unsigned int	gunDeltaRGB;

int16_t gunIHisto[255];    // Histogram for intensity, 255 levels.
unsigned int gunAverageLuminance = 0;

#define		_CAMERA_READY			1	// 1 = Camera module ready.
#define		_CAMERA_NOT_READY		0	// 0 = Camera module not ready.
int				gnCameraReady = _CAMERA_NOT_READY;		
										

// PRIVATE FUNCTION PROTOTYPES
inline int Min(int a, int b) {return (a < b)? a: b;}
inline int Max(int a, int b) {return (a > b)? a: b;}

///
/// Function name		: Proce_TCM8230_Driver
///
/// Author				: Fabian Kung
///
/// Last modified		: 13 Oct 2016
///
/// Code Version		: 1.00
///
/// Processor			: ARM Cortex-M4 family
///
/// Processor/System Resource
/// PINS				: 1. Pin PB14 (Output) = Camera reset.
///						  2. Pin PB13 (Output) = PCK0 output, clock to camera.
///                       3. Pin PA24-PA31 (Inputs) = PIODC0 to PIODC7 (Data0-7 of camera)
///                       4. Pin PA23 (Input) = PIODCCLK (Dclk)
///                       5. Pin PA15 (Input) = PIODCEN1 (VSysnc)
///                       6. Pin PA16 (Input) = PIODCEN2 (HSync)
///                       7. Pin PA12 (Output) = Camera LED control, active high.
///
/// MODULES				: 1. TWI0 (Two-wire interface 0, internal) and associated driver.
///                       2. PCK0 (Programmable clock 0, internal).
///                       3. PDC (Peripheral DMA Controller, internal).
///
/// RTOS				: Ver 1 or above, round-robin scheduling.
///
/// Global Variables    : gnFrameCounter

#ifdef __OS_VER			// Check RTOS version compatibility.
#if __OS_VER < 1
#error "Proce_TCM8230_Driver: Incompatible OS version"
#endif
#else
#error "Proce_TCM8230_Driver: An RTOS is required with this function"
#endif

///
/// Description	:
/// 1. This process initializes, control and retrieves the raw digital data from
/// the CMOS camera module TCM8230 made by Toshiba Corp. in 2004.  The TCM8230MD is a 
/// VGA (640x480) color CMOS camera module.  Clock to the camera module is provided by
/// on-chip peripheral PCK0.  At present the output of PCK0 can be set to 4 MHz or
/// 8 MHz, corresponding to camera frame rate of 5 fps and 10 fps.  For default we are using
/// 10 fps.
/// 2. DMA (Direct memory access) transfer is used to store the camera pixel data in the 
/// SRAM of the micro-controller.  Here each pixel data is represented by unsigned 32 bits integer,
/// in the array gunImgAtt[][].  The global variables gnImageWidth and gnImageHeight keep 
/// tracks of the width and height of each image frame.
/// 3. The global variable gnFrameCounter keeps track of the number of frame captured since
/// power on.  Other process can use this information to compare difference between frames.
/// 4. Pre-processing of the pixels color output to extract the Luminance, Contrast, Hue and 
/// Saturuation is performed here, and the result for each pixel is stored in gunImgAtt[x][y].
/// Where the x and y index denotes the pixel's location in the frame.

// User to edit these:
#define     _CAMERA_I2C2_ADD        60      // Camera I2C slave address.
#define     _CAMERA_POR_DELAY_MS    50		// Power on reset delay for camera in msec.
#define     _CAMERA_RESET_DELAY_MS  2		// Delay after disable the RESET pin in msec.

#define	PIN_CAMRESET_SET        PIOB->PIO_ODSR |= PIO_ODSR_P14		// Set camera Reset pin, PB14.
#define	PIN_CAMRESET_CLEAR		PIOB->PIO_ODSR &= ~PIO_ODSR_P14		// Clear camera Reset pin, PB14.

void Proce_TCM8230_Driver(TASK_ATTRIBUTE *ptrTask)
{
	int nTemp; 
	int nTemp2;
	static int nLineCounter = 0;
	
	// Variables associated with image pre-processing.
	int nR5, nG6, nB5;
	int nR6, nB6;
	int nLuminance;
	int ncolindex;
	int nHue;
	unsigned int unSat;
	unsigned int unMaxRGB, unMinRGB;
	int nDeltaRGB;
	int	nLuminance1, nLuminance2, nLuminance3, nLuminance4, nLuminance5, nLuminance6;
	int nLuminance7, nLuminance8;
	int	nLumGradx, nLumGrady, nLumGrad;
	static unsigned int unLumCumulative;
	
	static int nInitCounter = 0;


	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - Initialization of I2C module, Parallel Capture and PCK0.  The PCK0 provides the clock signal	
					//           to the camera, it's frequency will determine the frame rate of the camera. The I2C module controls
					//           the camera characteristics.  Note that upon power up the camera is already in reset state by 
					//			 virtue of the all IO pins of the micro-controller is set to 0 output state.
				
				
				// ----------------------------------------------------------------------------------------------------------------------------------	
				// --- NOTE: 4 April 2017 - Workaround for error in MVM 1 PCB ---
				// This is just a work-around for a mistake in the PCB artwork, where I mistakenly connect VSYNC pin to PA17, actually it should be
				// PA15.  Thus to disable this pin we set it to input, and PA17 is not used.  
				// These codes can be removed if we are using the correct PCB artwork.
				// Setup pin PA17 as input. General purpose input.
				PIOA->PIO_PPDDR |= PIO_PPDDR_P17;  // Disable internal pull-down to PA17.
				PIOA->PIO_PUER |= PIO_PUER_P17; // Enable internal pull-up to PA17.
				//PIOA->PIO_PER |= PIO_PER_P17;	// PA17 is controlled by PIO.
				PIOA->PIO_ODR |= PIO_ODR_P17;	// Disable output write to PA17.
				//PIOA->PIO_IFER |= PIO_IFER_P17;	// Enable input glitch filter to PA17. This is optional.
				// --- End of workaround code ---
				// -----------------------------------------------------------------------------------------------------------------------------------
				

				//nInitCounter = 0;							// Initialize "Initialization" counter.
				PIN_CAMRESET_SET;							// Disable camera RESET.	
				
				// Parallel Capture mode interrupt settings.
				 PIOA->PIO_PCIDR = PIOA->PIO_PCIDR | PIO_PCISR_DRDY | PIO_PCIDR_ENDRX | PIO_PCIDR_DRDY | PIO_PCIDR_RXBUFF | PIO_PCIDR_OVRE; // Disable all PCM interrupts.
				//PIOA->PIO_PCIDR = PIOA->PIO_PCIDR | PIO_PCISR_DRDY | PIO_PCIDR_DRDY | PIO_PCIDR_RXBUFF | PIO_PCIDR_OVRE; // Disable PCM interrupts.
				//PIOA->PIO_PCIER = PIOA->PIO_PCIER | PIO_PCIER_ENDRX;	// Enable end of reception transfer interrupt.
				
				// Peripheral DMA Controller for PIOA.
				// Initialize PDC PIOA receive operation.
				PDC_PIOA->PERIPH_RPR = gn16Pixel;				// Set receive buffer start address.
				PDC_PIOA->PERIPH_RCR = gnImageWidth;			// Set receive buffer length.				
				PDC_PIOA->PERIPH_RNPR = PDC_PIOA->PERIPH_RPR;	// Set next receive buffer start address.
				PDC_PIOA->PERIPH_RNCR = 0;						// Set next receive buffer length. 
				PDC_PIOA->PERIPH_PTCR = PDC_PIOA->PERIPH_PTCR | PERIPH_PTCR_RXTEN;	// Enable receiver transfer.
							
				// Enable parallel Capture mode of PIOA.
				// Note: 26 Nov 2015, the following sequence needs to be followed (from datasheet), setting of PCEN flag
				// should be the last.  Once PCEN is set, all the associated pins will automatically become input pins.
				// Note that upon power up, by default all pins are set to output, so momentarily there can be a high current
				// drawn from the chip.
				
				PIOA->PIO_PCMR = PIOA->PIO_PCMR | PIO_PCMR_DSIZE_HALFWORD;				// Data size is half word (16 bits).
				PIOA->PIO_PCMR = PIOA->PIO_PCMR & ~PIO_PCMR_ALWYS & ~PIO_PCMR_HALFS;	// Sample all data, sample the data only when both
																						//  PIODCEN1 or PIODCEN2 are HIGH.	
				PIOA->PIO_PCMR = PIOA->PIO_PCMR | PIO_PCMR_PCEN;						// Enable Parallel Capture module.  All related pins
																						// will be automatically set to input.
																						
				
				// Settings of PCK0 (Programmable Clock 0).			
				
																// Set Programmable Clock 0 (PCK0) to:
																// Source: Main clock (8 MHz).
																// Pre-scaler = divide by 2.
																// Effective output = 4 MHz.			
				//PMC->PMC_PCK[0] = PMC->PMC_PCK[0] | PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES_CLK_2;
					
																// Set Programmable Clock 0 (PCK0) to:
																// Source: Main clock (8 MHz).
																// Pre-scaler = divide by 1.
																// Effective output = 8 MHz.				
				PMC->PMC_PCK[0] = PMC->PMC_PCK[0] | PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES_CLK_1;	
				PIOB->PIO_PDR = (PIOB->PIO_PDR) | PIO_PDR_P13;	// Set PB13 to be controlled by Peripheral.
				PMC->PMC_SCER = PMC->PMC_SCER | PMC_SCER_PCK0;	// Enable PCK0.	
				PIOB->PIO_ABCDSR[0] = (PIOB->PIO_ABCDSR[0]) | PIO_ABCDSR_P13;	// Connect PB13 to Peripheral
																				// block B.	
				gnFrameCounter = 0;								// Clear frame counter.			
				OSSetTaskContext(ptrTask, 1, 500*__NUM_SYSTEMTICK_MSEC); // A long delay for the power supply to settle down.
																// Next state = 1, timer = 500 msec.
			break;
			
			case 1: // State 1 - Reset camera.
				PIN_CAMRESET_CLEAR;							// Reset camera (Active low).
				OSSetTaskContext(ptrTask, 2, _CAMERA_POR_DELAY_MS*__NUM_SYSTEMTICK_MSEC);
															// Next state = 2, timer = Camera reset delay.
			break;

			case 2: // State 2 - Disable camera reset, delay.
				PIN_CAMRESET_SET;							// Disable camera RESET.		
				OSSetTaskContext(ptrTask, 3, _CAMERA_RESET_DELAY_MS*__NUM_SYSTEMTICK_MSEC);
															// Next state = 3, timer = Camera reset delay.			
			break;
			
			case 3: // State 3 - Set synchronization code, vertical and horizontal timing format, and picture mode.
			//nWriteSCCB(0x1E, 0x6C);                 // DMASK = 0x01.
			// HSYNCSEL = 1.
			// CODESW = 1 (Output synchronization code).
			// CODESEL = 0 (Original synchronization code format).
			// TESPIC = 1 (Enable test picture mode).
			// PICSEL = 0x00 (Test picture output is color bar).
			
			//nWriteSCCB(0x1E, 0x68);                 // DMASK = 0x01.
			// HSYNCSEL = 1.
			// CODESW = 1 (Output synchronization code).
			// CODESEL = 0 (Original synchronization code format).
			// TESPIC = 0 (Disable test picture mode).
			// PICSEL = 0x00 (Test picture output is color bar).
			//OSSetTaskContext(ptrTask, 3, 20);
				if (gI2CStat.bI2CBusy == 0)				// Make sure I2C module is not being used.
				{
					gbytI2CByteCount = 1;				// Indicate no. of bytes to transmit.
					gbytI2CRegAdd = 0x1E;				// Start address of register.
					gbytI2CTXbuf[0] = 0x68;				// Data.
					gbytI2CSlaveAdd =  _CAMERA_I2C2_ADD;	// Camera I2C slave address.
					gI2CStat.bSend = 1;
					OSSetTaskContext(ptrTask, 4, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 4, timer = 5 msec.
				}
				else
				{
					OSSetTaskContext(ptrTask, 3, 1);		// Next state = 3, timer = 1.
				}
			break;
			
			case 4: // State 4 - Set AC frequency (ACF) of fluorescent light to 50 Hz for Malaysia.  Actually this is not important if the
					// ACFDET bit is set to AUTO (0).
					// Also maximum frame rate is set to 30 fps.
					// DCLK polarity = Normal.
				if (gI2CStat.bI2CBusy == 0)				// Make sure I2C module is not being used.
				{
					gbytI2CByteCount = 1;				// Indicate no. of bytes to transmit.
					gbytI2CRegAdd = 0x02;				// Start address of register.
					gbytI2CTXbuf[0] = 0x00;				// Data.
					gbytI2CSlaveAdd =  _CAMERA_I2C2_ADD;	// Camera I2C slave address.
					gI2CStat.bSend = 1;
					OSSetTaskContext(ptrTask, 5, 5*__NUM_SYSTEMTICK_MSEC);      // Next state = 5, timer = 5 msec.
				}
				else
				{
					OSSetTaskContext(ptrTask, 4, 1);		// Next state = 4, timer = 1.
				}
			break;

			case 5: // State 5 - Turn on camera and set output format and frame resolution.	
					// 28 Jan 2016: This step should be done last.  After the camera is turned ON, the signal lines
					// DCLK, HSYNC, VSYNC will become active.	
				// Turn on camera.
				// Enable D0-D7 outputs.
				// RGB565 format, color.
				if (gI2CStat.bI2CBusy == 0)				// Make sure I2C module is not being used.
				{
				     gbytI2CByteCount = 1;				// Indicate no. of bytes to transmit.
				     gbytI2CRegAdd = 0x03;				// Start address of register.
				     gbytI2CTXbuf[0] = 0x0E;			// Data, QQVGA(f).
					 //gbytI2CTXbuf[0] = 0x12;			// Data, QQVGA(z).
				     gbytI2CSlaveAdd =  _CAMERA_I2C2_ADD;	// Camera I2C slave address.
				     gI2CStat.bSend = 1;
					
					 
					 // Enable PCK0 (programmable clock output 0).  This supply the clock input to the camera.
					 // Set Programmable Clock 0 (PCK0) to:
					 // Source: Main clock (8 MHz).
					 // Pre-scaler = divide by 1.
					 // Effective output = 8 MHz.
					 PMC->PMC_PCK[0] = PMC->PMC_PCK[0] | PMC_PCK_CSS_MAIN_CLK | PMC_PCK_PRES_CLK_1;
					 //PIOB->PIO_ABCDSR[0] = (PIOB->PIO_ABCDSR[0]) | PIO_ABCDSR_P13;	// Connect PB13 to Peripheral
					 //PMC->PMC_SCER = PMC->PMC_SCER | PMC_SCER_PCK0;	// Enable PCK0.	

					 // Note: 5 April 2017 - I discovered that the camera module sometimes would not initialize properly,
					 // could be due to the reset timing and duration.  Since I do not have the exact datasheet for the
					 // camera module (only the preliminary can be found online), I figure the easiest way to overcome this
					 // is to execute the initialization sequence multiple times.  Here we initialize the camera module 
					 // twice.
					 nInitCounter++;						
					 if (nInitCounter == 10)											// Have we initialize the camera module
					 {																// 5x?
						OSSetTaskContext(ptrTask, 6, 5*__NUM_SYSTEMTICK_MSEC);      // Yes, next state = 6, timer = 5 msec.
					 }
					 else
					 {
						OSSetTaskContext(ptrTask, 1, 10*__NUM_SYSTEMTICK_MSEC);     // No, next state = 1, timer = 10 msec.
						//OSSetTaskContext(ptrTask, 0, 10*__NUM_SYSTEMTICK_MSEC);     // No, next state = 0, timer = 10 msec.
					 }

				}		
				else                                         // I2C still busy.     
				{
					OSSetTaskContext(ptrTask, 5, 1);		// Next state = 5, timer = 1.
				}
			break;
			
			case 6: // State 6 - Wait for idle condition, this is signified by:
					//           VSync (or VD) = 'H'
					//           HSync (or HD) = 'L' 
					//           to indicate start of new frame.  So we wait for this H-to-L transition before
					//			 we enable the PDC.
				// Check pin PA15 (PIODCEN1) and PA16 (PIODCEN2) status.

				gnCameraReady = _CAMERA_READY;				// Indicates camera module is ready.

				if (((PIOA->PIO_PDSR & PIO_PDSR_P15) > 0) && ((PIOA->PIO_PDSR & PIO_PDSR_P16) == 0))
				{	// Idle condition
					OSSetTaskContext(ptrTask, 7, 1);		// Next state = 7, timer = 1.
				}
				else
				{	// Non-idle condition.				
					OSSetTaskContext(ptrTask, 6, 1);		// Next state = 6, timer = 1.
				} 			
			break;
			
			case 7: // State 7 - Wait for start of new frame condition, this is signified by:
					//			 Vsync = 'L'
					//			 Hsync = 'L'
					
				// Check pin PA15 status (PIODCEN1).
				if ((PIOA->PIO_PDSR & PIO_PDSR_P15) == 0) 
				{	// Low.			
					// Initialize PDC PIOA receive operation.		
					PDC_PIOA->PERIPH_RPR = gn16Pixel;				// Set receive buffer start address.
					PDC_PIOA->PERIPH_RCR = gnImageWidth;			// Set receive buffer length.
					PDC_PIOA->PERIPH_RNPR = PDC_PIOA->PERIPH_RPR;	// Set next receive buffer start address.
					PDC_PIOA->PERIPH_RNCR = 0;						// Set next receive buffer length.
					
					nLineCounter = 0;								// Reset line counter.
					OSSetTaskContext(ptrTask, 8, 1);				// Next state = 8, timer = 1.
				}
				else
				{	// High.
					OSSetTaskContext(ptrTask, 7, 1);				// Next state = 7, timer = 1.
				}
				break;			

			case 8: // State 8 - Wait until DMA transfer of 1 line of pixel data is complete.
				
				//PIOA->PIO_ODSR &= ~PIO_ODSR_P19;					// Clear flag.
				
				if ((PIOA->PIO_PCISR & PIO_PCISR_ENDRX) > 0)		// Check if pixel line buffer is full.
				{
																	// Pixel line buffer is full.
					nLineCounter++;									// Increment row counter.
					// Initialize PDC PIOA receive operation.
					PDC_PIOA->PERIPH_RPR = gn16Pixel;				// Set receive buffer start address.
					PDC_PIOA->PERIPH_RCR = gnImageWidth;			// Set receive buffer length.
					PDC_PIOA->PERIPH_RNPR = gn16Pixel;				// Set next receive buffer start address.
					PDC_PIOA->PERIPH_RNCR = 0;						// Set next receive buffer length.		
					
					
					// --- Pre-processing image data here ---
					// Extract intensity.
					for (ncolindex = 0; ncolindex < gnImageWidth; ncolindex++)
					{				
						// Compute the 8-bits grey scale or intensity value of each pixel and
						// update grey scale histogram.
						nTemp2 = gn16Pixel[ncolindex];
						
						// Assume data read into parallel bus by ARM Cortex M4 is: [Byte0][Byte1]
						
						nTemp = (nTemp2 << 8) & 0xFF00;		// Swap the position of lower and upper 8 bits!
						nTemp2 = (nTemp2 >> 8) & 0xFF;		// This is due to the way the PDC store the pixel data.  
															// 14 Jan 2016: The upper 16 bits may not be 0! Thus need
															// to mask off. 
						nTemp = nTemp + nTemp2;

						// Assume data read into parallel bus by ARM Cortex M4 is: [Byte1][Byte0]
						//nTemp = nTemp2;
						
						nR5 = (nTemp >> 11) & 0x1F;			// Get the 5 bits R component.
						nG6 = (nTemp >> 5) & 0x3F;			// Get the 6 bits G component.
						nB5 = nTemp & 0x01F;				// Get the 5 bits B component.

						// --- 6 Jan 2015 ---
						// Here we approximate the luminance I (or Y) as:
						// I = 0.250R + 0.625G + 0.125B = (2R + 5G + B)/8
						// where R, G and B ranges from 0 to 255.
						// or I = (2*R + 5*G + B)>>3 for integer variables R, G, B.
						// This avoids multiplication with real number, hence speeding up the
						// computation, though with the increase in blue and decrease
						// in green components intensities, we would get a slightly darker
						// grey scale image.
						
						//nLuminance = (2*(nR5<<3) + 5*(nG6<<2) + (nB5<<3))>>3;
						
						// Alternatively since actual values for R, G and B are 5, 6 and 5 bits respectively,
						// we can normalize R and B to 6 bits by shifting, then convert to gray scale as:
						//nLuminance = (2*(nR5<<1) + 5*(nG6) + (nB5<<1))>>2;
						
						//nLuminance = (3*(nR5<<1) + 2*(nG6) + 3*(nB5<<1))>>2;	// 15 Jan 2016: Test, where we deemphasize the green component.  
																				// There seems to be some sampling noise in the green component.
						
						// Or more efficient alternative, with no multiplication:
						nLuminance = ((nR5<<2) + (nG6<<2) + (nB5<<1) + nG6)>>2;
						// This will result in gray scale value from 0 to 127, occupying 7 bits.
						
						//nLuminance = ((nR5<<2) + (nG6<<1) + (nB5<<2) + nR5 + nB5)>>2;	// 15 Jan 2016: Test, where we deemphasize the green component.
						//nLuminance = ((nR5<<2) + (nG6<<1) + (nB5<<2) + nR5 + nB5)>>1;	// 27 Jan 2016: Same as above, but result in 8 bits luminance value.
						// There seems to be some sampling noise in the green component.
						
						//nLuminance = (nR5<<1) + (nG6>>2) + nB5;								// 12 March 2016: Test.  Reduce further the contribution of Green
						
						
						unLumCumulative = unLumCumulative + nLuminance;							// Update the sum of luminance for all pixels in the frame.
						
						gunIHisto[nLuminance]++;												// Update the intensity histogram.
						
						
						
						// Optional - Reduce the luminance to black and white.
						/*
						nLumReference = 100;									// Agreed luminance for White.
						if (nLuminance < (gunAverageLuminance))
						{
							nLuminance = 0;
						}
						else 
						{
							nLuminance = 100;
						}
						*/
				
												
						nR6 = nR5<<1;								// Since R and B are 5 bits, we normalize them
						nB6 = nB5<<1;								// to 6 bits so that they are comparable to the
																	// G components.

						// --- Compute the saturation level ---
						unMaxRGB = Max(nR6, Max(nG6, nB6));     // Find the maximum of R, G or B component
						unMinRGB = Min(nR6, Min(nG6, nB6));     // Find the minimum of R, G or B components.
						nDeltaRGB = unMaxRGB - unMinRGB;
						unSat = nDeltaRGB;                         
						// Note: Here we define the saturation as the difference between the maximum and minimum RGB values.
						// In normal usage this value needs to be normalized with respect to maximum RGB value so that
						// saturation is between 0.0 to 1.0.  Here to speed up computation we avoid using floating point
						// variables. Thus the saturation is 6 bits since the color components are 6 bits, from 0 to 63.

						// --- Compute the hue ---
						// When saturation is too low, the color is near gray scale, and hue value is not accurate.
						// Similarly when the light is too bright, the difference between color components may not be large
						// enough to work out the hue, and hue is also not accurate.
						if (nDeltaRGB < 6)               
						{                                       // The minimum RGB component intensity corresponds
							nHue = _NO_HUE;                     // to the white level intensity.  Thus the difference
						}		                                // between maximum RGB component and white level is an
																// indication of the saturation level.  This difference
																// needs to be sufficiently large for reliable hue 
																// recognition, we arbitrary sets this to at least 10% 
																// of the maximum RGB component.  For 6 bits RGB color
																// components, max = 63, min = 0.  Thus maximum difference
																// is 63.  10% of this is 6.30.
						else
						{
							if (nR6 == unMaxRGB)          // nR6 is maximum. Note: since we are working with integers,
							{                                          // be aware that when we perform integer division,
								nHue = (60*(nG6 - nB6))/nDeltaRGB;   // the remainder will be discarded.
							}
							else if (nG6 == unMaxRGB)     // nG6 is maximum.
							{
								nHue = 120 + (60*(nB6 - nR6))/nDeltaRGB;
							}
							else if (nB6 == unMaxRGB)     // nB6 is maximum.
							{
								nHue = 240 + (60*(nR6 - nG6))/nDeltaRGB;
							}
		                 
							if (nHue < 0)
							{
								nHue = nHue + 360;
							}
						}
						
						// Test (12 Dec 2016)
						if (nLineCounter == 60)
						{
							if (ncolindex == 80)
							{
								gunR = nR5;
								gunG = nG6;
								gunB = nB5;
								gunDeltaRGB = nDeltaRGB;
							}
						}
						
						// Computing the luminance gradient using Sobel's Kernel.
						if ((ncolindex > 1) && (nLineCounter > 1))						// See notes on the derivation of this range.
						{
							// Note: 13 Oct 2016, need to improve the efficiency.  In theory for each new pixel we only need
							// to fetch 4 pixel points only.
							nLuminance1 = gunImgAtt[ncolindex-2][nLineCounter-2] & _LUMINANCE_MASK;			
							nLuminance2 = gunImgAtt[ncolindex][nLineCounter-2] & _LUMINANCE_MASK;	
							nLuminance3 = gunImgAtt[ncolindex-2][nLineCounter-1] & _LUMINANCE_MASK;
							nLuminance4 = gunImgAtt[ncolindex][nLineCounter-1] & _LUMINANCE_MASK;
							nLuminance5 = gunImgAtt[ncolindex-2][nLineCounter] & _LUMINANCE_MASK;
							//nLuminance6 = gunImgAtt[ncolindex][nLineCounter] & _LUMINANCE_MASK;	// This is the same as nLuminance.					
							nLuminance6 = nLuminance;	
							nLuminance7 = gunImgAtt[ncolindex-1][nLineCounter-2] & _LUMINANCE_MASK;
							nLuminance8 = gunImgAtt[ncolindex-1][nLineCounter] & _LUMINANCE_MASK;							
							
							// Calculate x gradient											
							nLumGradx = nLuminance2  + nLuminance6 - nLuminance1 - nLuminance5;
							nLumGradx = nLumGradx + ((nLuminance4 - nLuminance3) << 1);
							// Calculate y gradient
							nLumGrady = nLuminance5  + nLuminance6 - nLuminance1 - nLuminance2;
							nLumGrady = nLumGradx + ((nLuminance8 - nLuminance7) << 1);

							if (nLumGradx < 0)		// Only magnitude is required.
							{
								nLumGradx = -nLumGradx;
							}
							if (nLumGrady < 0)		// Only magnitude is required.
							{
								nLumGrady = -nLumGrady;
							}
																// Calculate the magnitude of the luminance gradient.
							nLumGrad = nLumGradx + nLumGrady;	// It should be nLumGrad = sqrt(nLumGradx^2 + nLumGrady^2)
																// Here we the approximation nLumGrad = |nLumGradx| + |nLumGrady|

							if (nLumGrad > 127)		// Limit the maximum value to 127 (7 bits only).  Bit8 is not used for
							{						// luminance indication.
								nLumGrad = 127;
							}
							if (nLumGrad < 60)		// To remove gradient noise.  Can reduce to 10 if the camera quality is good.
							{
								nLumGrad = 0;
							}
						}
						else
						{
							nLumGrad = 0;
						}

						
						// Update the pixel attributes.
								
						// Test
						//gunImgAtt[ncolindex][nLineCounter] = nR6<<1;						// Send red component (6 bits).
						//gunImgAtt[ncolindex][nLineCounter] = nG6<<1;						// Send Green component (6 bits).
						//gunImgAtt[ncolindex][nLineCounter] = nB6<<1;						// Send Blue component (6 bits).
						
						// Consolidate all the pixel attribute into the attribute array.
						//gunImgAtt[ncolindex][nLineCounter] = nLuminance | (nLumGrad << _GRAD_SHIFT) | (unSat << _SAT_SHIFT);
						gunImgAtt[ncolindex][nLineCounter] = nLuminance | (nLumGrad << _GRAD_SHIFT) | (unSat << _SAT_SHIFT) | (nHue << _HUE_SHIFT);
						
					}				
					
					// --- End of pre-processing image data here ---
													
				}
				else
				{
					//PIOA->PIO_ODSR &= ~PIO_ODSR_P19;			// Clear flag.
				}
				
				if (nLineCounter == gnImageHeight)				// Check for end of frame.
				{
					
					OSSetTaskContext(ptrTask, 9, 1);			// Next state = 9, timer = 1.
				}	
				else
				{						
																// Not end of frame yet, continue to receive next 
																// line of pixel data.
					OSSetTaskContext(ptrTask, 8, 1);			// Next state = 8, timer = 1.
				}						
			break;

			case 9: // State 9 - End, do some tidy-up chores if necessary.
				gnFrameCounter++;								// Update frame counter.
				for (nTemp = 0; nTemp<255; nTemp++)				// Clear the luminance histogram array.
				{
					gunIHisto[nTemp] = 0;
				}
				gunAverageLuminance = unLumCumulative/_NOPIXELSINFRAME;	// Update the average Luminance.
				unLumCumulative = 0;							// Reset the sum of Luminance.
				OSSetTaskContext(ptrTask, 6, 1);				// Next state = 6, timer = 1.
			break;
			
			default:
				OSSetTaskContext(ptrTask, 0, 1);				// Back to state = 0, timer = 1.
			break;
		}
	}
}

///
/// Process name		: Proce_Camera_LED_Driver
///
/// Author              : Fabian Kung
///
/// Last modified		: 28 July 2017
///
/// Code Version		: 0.90
///
/// Processor			: ARM Cortex-M4 family
///
/// Processor/System Resources
/// PINS                : Pin PA12 (Output) = Camera LED control, active high.	
///						  Pin PA20 (Output) = Right Eye LED control, active high.
///						  Pin PA22 (Output) = Left Eye LED control, active high.
///
/// MODULES             :
///
/// RTOS                : Ver 1 or above, round-robin scheduling.
///
/// Global variables    : gnCameraLED

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
#if 			  __OS_VER < 1
#error "Proce_Camera_LED_Driver: Incompatible OS version"
#endif
#else
#error "Proce_Camera_LED_Driver: An RTOS is required with this function"
#endif

/// Description: Process to control the eye LEDs and camera LED.
/// Usage:
/// To light up the visible light LED, set gnCameraLED, gnEyeLEDLeft and gnEyeLEDRight from 1 to 6 (6 being the brightest).
/// Setting gnCameraLED, gnEyeLEDLeft and gnEyeLEDRight to 0 or smaller turn off the LED. Setting the respective LED control
/// register to greater than 6 is similar to maintaining brightest intensity.
/// 
#define	PIN_EYELED_RIGHT_ON		PIOA->PIO_ODSR |= PIO_ODSR_P20		// Set pin PA20.
#define	PIN_EYELED_RIGHT_OFF	PIOA->PIO_ODSR &= ~PIO_ODSR_P20		// Clear pin PA20.
#define	PIN_EYELED_LEFT_ON		PIOA->PIO_ODSR |= PIO_ODSR_P22		// Set pin PA22.
#define	PIN_EYELED_LEFT_OFF		PIOA->PIO_ODSR &= ~PIO_ODSR_P22		// Clear pin PA22.
#define	PIN_CAMLED_ON			PIOA->PIO_ODSR |= PIO_ODSR_P12		// Turn on camera LED, PA12.
#define	PIN_CAMLED_OFF			PIOA->PIO_ODSR &= ~PIO_ODSR_P12		// Turn off camera LED, PA12.

void Proce_Camera_LED_Driver(TASK_ATTRIBUTE *ptrTask)
{
	static int	nCounter = 0;

	if (ptrTask->nTimer == 0)
	{
		switch (ptrTask->nState)
		{
			case 0: // State 0 - Initialization.
				PIN_CAMLED_OFF;							// Off the LED first.
				PIN_EYELED_RIGHT_OFF;
				PIN_EYELED_LEFT_OFF;
				gnCameraLED = 0;
				gnEyeLEDRight = 0;
				gnEyeLEDLeft = 0;
				OSSetTaskContext(ptrTask, 1, 1000*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 1000 msec.
				break;

			case 1: // State 1 - Wait for camera module to complete initialization sequence.
				if (gnCameraReady == _CAMERA_READY)
				{
					OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
				}
				else
				{
					OSSetTaskContext(ptrTask, 1, 1);    // Next state = 1, timer = 1.
				}
			break;

			case 2: // State 2 - Drive the eye LED.
				if (gnCameraLED > nCounter)
				{
					PIN_CAMLED_ON;
				}
				else
				{
					PIN_CAMLED_OFF;
				}
				if (gnEyeLEDRight > nCounter)
				{
					PIN_EYELED_RIGHT_ON;
				}
				else
				{
					PIN_EYELED_RIGHT_OFF;
				}
				if (gnEyeLEDLeft > nCounter)
				{
					PIN_EYELED_LEFT_ON;
				}
				else
				{
					PIN_EYELED_LEFT_OFF;
				}
				nCounter++;
				if (nCounter == 6)
				{
					nCounter = 0;
				}
				OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
				break;
			
			default:
				OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
				break;
		}
	}
}