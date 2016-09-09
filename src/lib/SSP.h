#ifndef SSP_H
#define SSP_H

//Definitions for PushTracker

#define SPIF	(1<<0)
#define SPI_COMMAND_WRITE          0x00
#define SPI_COMMAND_READ           0x01
#define SPI_COMMAND_WRITE_AND_READ 0x02
#define FAILSAFE_VALUE 5000

char SSP_send_recv(char c);
void SSP_send(char c);
char SSP_recv(void);
void initSSP(void);

#endif 
