#include "main.h"
#include "SSP.h"
#include "LPC214x.h"


char SSP_send_recv(char c)
{
	SSPDR = c;
	while ((SSPSR & (1<<4)));		// while SSP is busy
	return SSPDR;
}

void SSP_send(char c)
{
	SSP_send_recv(c);
}

char SSP_recv(void)
{
	char _dummy;
	_dummy = SSP_send_recv(0xFF);
	return _dummy;
}

/*****************************************************************************
 *
 * Description:
 *    Initialize the SPI interface to use interrupts.
 *
 ****************************************************************************/
void
initSSP(void)
{ 
	unsigned int _dummy;
	PINSEL1 &= ~(3<<8);		// put CS pin into GPIO
	#ifdef USE_SLOW_GPIO
		IODIR0 |= SPI_SLAVE_CS;
		IOSET0 = SPI_SLAVE_CS;
	#else
		FIO0DIR |= SPI_SLAVE_CS;
		FIO0SET = SPI_SLAVE_CS;
	#endif
	PINSEL1 |= (1<<3)|(1<<5)|(1<<7); // Enable SCLK1 on P0.17,Enable MISO1 on P0.18,Enable MOSI1 on P0.19
	PINSEL1 &= ~((1<<2)|(1<<4)|(1<<6));
	
	//initialize SPI interface
	SSPCPSR = 12;    // 5 MHz spi interface clk
	SSPCR0 = (0<<8)|(1<<7)|(1<<6)|(00<<4)|(7<<0);   // 0 SCR (multiplier for prescaler), CPHA = 0, CPOL = 1, SPI frame format, 8 bit data transfer
	//SSPCR1 = (0<<2);		// Master mode
	SSPCR1 |= (1<<1);		// turn on SSP
	int i;
	for (i=0;i<8;i++)
	{
		_dummy = SSPDR;		// clears the FIFO buffer
	}
}
