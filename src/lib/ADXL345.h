#ifndef ADXL345_H
#define ADXL345_H

//**********************************************************
//
//                  Pin Definitions
//
//**********************************************************


//**********************************************************
//
//                  Register Addresses
//
//**********************************************************
#define POWER_CTL	0x2D
//****************************
//POWER_CTL BITS
#define	WAKE_0		(1<<0)
#define WAKE_1		(1<<1)
#define PD			(1<<2)
#define MEASURE 	(1<<3)
#define AUTO_SLEEP	(1<<4)
#define LINK		(1<<5)
//****************************
#define OutX		0x32
#define OutY		0x34
#define	OutZ		0x36

#define READ_1	0x80
#define READ_2	0xC0
#define	WRITE	0x00

//**********************************************************
//
//                  	Macros
//
//**********************************************************

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
// Tested value : 248
#define GRAVITY 248  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

void initAccel(void);
short accelX(void);
short accelY(void);
short accelZ(void);
struct accel_struct accel_all(void);
void powerdownAccel(void);

#endif
