
extern void (vButtonISRWrapper) ( void );

	/* Just to stop compiler warnings. */
	( void ) pvParameters;

	/* Configure the interrupt. */
	portENTER_CRITICAL();
	{
		/* Configure P0.14 to generate interrupts. */
		PINSEL0 |= mainP0_14__EINT_1;
		EXTMODE = mainEINT_1_EDGE_SENSITIVE;
		EXTPOLAR = mainEINT_1_FALLING_EDGE_SENSITIVE;

		/* Setup the VIC for EINT 1. */
		VICIntSelect &= ~mainEINT_1_VIC_CHANNEL_BIT;
		VICIntEnable |= mainEINT_1_VIC_CHANNEL_BIT;
		VICVectAddr1 = ( long ) vButtonISRWrapper;
		VICVectCntl1 = mainEINT_1_ENABLE_BIT | mainEINT_1_CHANNEL;
	}
	portEXIT_CRITICAL();



void vButtonISRWrapper( void ) __attribute__ ((naked));
void vButtonHandler( void ) __attribute__ ((noinline));

void vButtonHandler( void )
{
extern xSemaphoreHandle xButtonSemaphore;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( xButtonSemaphore, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken )
	{
		/* We have woken a task.  Calling "yield from ISR" here will ensure
		the interrupt returns to the woken task if it has a priority higher
		than the interrupted task. */
		portYIELD_FROM_ISR();
	}

    EXTINT = isrCLEAR_EINT_1;
    VICVectAddr = 0;
}
/*-----------------------------------------------------------*/

void vButtonISRWrapper( void )
{
	/* Save the context of the interrupted task. */
	portSAVE_CONTEXT();

	/* Call the handler to do the work.  This must be a separate function to
	the wrapper to ensure the correct stack frame is set up. */
	__asm volatile( "bl vButtonHandler" );

	/* Restore the context of whichever task is going to run once the interrupt
	completes. */
	portRESTORE_CONTEXT();
}




















#ifndef MAIN_H
#define MAIN_H

//#include <stdlib.h>
#include "LPC214x.h"


//#define DEBUG	1		// used for printing to the serial terminal out of the programming pins

//================================
//		Boolean Defines
//================================
#define ON	0
#define OFF	1
#define TRUE 1
#define FALSE 0

//================================
//			I/O Defines
//================================
//#define USE_SLOW_GPIO
#define USE_FAST_GPIO


#define CHIP_SELECT	(1<<20)
#define SPI_SLAVE_CS (1<<20)  //pin P0.20

#ifdef USE_SLOW_GPIO
	#define SelectAccel()	IOCLR0 = CHIP_SELECT
	#define UnselectAccel()	IOSET0 = CHIP_SELECT
#else
	#define SelectAccel()	FIO0CLR = CHIP_SELECT
	#define UnselectAccel()	FIO0SET = CHIP_SELECT
#endif



#define OLED_PWR	(1<<30)			// P0.30

#ifdef USE_SLOW_GPIO
	#define OLED_ON		IOSET0 = OLED_PWR
	#define OLED_OFF	IOCLR0 = OLED_PWR
#else
	#define OLED_ON		FIO0SET = OLED_PWR
	#define OLED_OFF	FIO0CLR = OLED_PWR
#endif

#define BLUE			21		// blue LED is on P0.21 (active low)
#define USB_LED			31		// usb (green) LED is on P0.31 (active low)
#define SWITCH_UP		10
#define SWITCH_ENTER	11
#define SWITCH_DOWN		12

#define ODOMETER_PIN	25		// D+ is on P0.26, and is used for odometer, when it goes low, the switch is actuated by a magnet

//================================
//		PushTracker Defines
//================================
#define LOG_INTERVAL		30
#define ODO_INTERVAL		5
#define WRITE_INTERVAL		2000
#define SYNC_INTERVAL		WRITE_INTERVAL

#define RAF_BUFFER_SIZE		5
#define VAL_BUFFER_SIZE		8
#define RAW_BUFFER_SIZE		25

struct accel_struct
{
	short accel_x;
	short accel_y;
	short accel_z;
};

struct PushTracker_struct
{
	unsigned int Pushes;
	unsigned int push_times[RAF_BUFFER_SIZE];

	unsigned int min_times[VAL_BUFFER_SIZE];
	unsigned int max_times[VAL_BUFFER_SIZE];
	float min_vals[VAL_BUFFER_SIZE];
	float max_vals[VAL_BUFFER_SIZE];
	unsigned char min_mask;
	unsigned char max_mask;
	float RAF_diff[RAF_BUFFER_SIZE];
	
	unsigned int time_buffer[RAW_BUFFER_SIZE];
	short raw_buffer[RAW_BUFFER_SIZE];
	float filtered_buffer[RAW_BUFFER_SIZE];
};

void delay_ms(unsigned int count);
void load_data(int time, int variable);
void reset_system(void);
void GraphAccelVals(void);
void LogData(void);
struct PushTracker_struct maxima(struct PushTracker_struct PT);
struct PushTracker_struct minima(struct PushTracker_struct PT);

#endif