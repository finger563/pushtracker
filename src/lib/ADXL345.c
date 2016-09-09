#include "ADXL345.h"
#include "SSP.h"
#include "main.h"

void initAccel(void)
{	
	unsigned char byte=0;
	
	SelectAccel();
	SSP_send(0x80);			// read Device ID from addr 0x00
	byte = SSP_recv();
	UnselectAccel();
	
	//rprintf("Device ID: %d\n",byte);
	
	while(byte != 0x28)
	{
		SelectAccel();
		SSP_send(0x2D);		
		SSP_send(0x28);
		UnselectAccel();
		
		delay_ms(5);
		
		SelectAccel();
		SSP_send(0xAD);		//read from power control register
		byte = SSP_recv();
		UnselectAccel();
		
		//rprintf("received %d\n",byte);
		
		delay_ms(2000);
	}
	
	SelectAccel();
	SSP_send(0x31);
	SSP_send(0x08);
	UnselectAccel();
	
	//rprintf("sent 0x31 and 0x08.\n");
	
	delay_ms(5);
	
	SelectAccel();
	SSP_send(0xB1);
	byte = SSP_recv();
	UnselectAccel();
	
	//rprintf("received %d.\n",byte);
	
	SelectAccel();
	SSP_send(0x38);
	SSP_send(0x00);
	UnselectAccel();
	
	//rprintf("sent 0x38 and 0x00.\n");
	
	delay_ms(5);
	
	SelectAccel();
	SSP_send(0xB8);
	byte = SSP_recv();
	UnselectAccel();
}

short accelX(void)
{
	short val=0;

	SelectAccel();
	SSP_send(READ_2 | OutX);
	val = SSP_recv();
	val |= (SSP_recv()<<8);
	UnselectAccel();
	
	return val;
}	

short accelY(void)
{	
	short val=0;
	
	SelectAccel();
	SSP_send(READ_2 | OutY);
	val = SSP_recv();
	val |= (SSP_recv()<<8);
	UnselectAccel();
	
	return val;
}

short accelZ(void)
{	
	short val=0;

	SelectAccel();
	SSP_send(READ_2 | OutZ);
	val = SSP_recv();
	val |= (SSP_recv()<<8);
	UnselectAccel();
	
	return val;
}

struct accel_struct accel_all(void)
{
	short _dummy;
	struct accel_struct func_struct;
	SelectAccel();
	SSP_send(READ_2| OutX);
	_dummy = SSP_recv();
	_dummy |= (SSP_recv()<<8);
	func_struct.accel_x = _dummy;
	_dummy = SSP_recv();
	_dummy |= (SSP_recv()<<8);
	func_struct.accel_y = _dummy;
	_dummy = SSP_recv();
	_dummy |= (SSP_recv()<<8);
	func_struct.accel_z = _dummy;
	UnselectAccel();
	
	return func_struct;
}

void powerdownAccel(void)
{
	SelectAccel();
	SSP_send(WRITE | POWER_CTL);
	SSP_send(PD);
	UnselectAccel();
}
