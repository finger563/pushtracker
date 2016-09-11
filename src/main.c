/*
  Logomatic V2 Firmware
  SparkFun Electronics 2008
        
  Basic, event based text logging
        
  When something happens, write some text to a file
 
*/

/* Header Files */
//======================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
//#include "LPC214x.h"

//UART0 Debugging
#include "serial.h"
#include "rprintf.h"

#include "OLED.h"
#include "ADXL345.h"
#include "SSP.h"
#include "menu.h"

//Needed for main function calls
#include "string_printf.h"
#include "fat16.h"
#include "rootdir.h"
#include "sd_raw.h"

/* Global Variables */
//======================================
#define ON      0
#define OFF     1

#define STAT1   21
#define STAT0   31

struct fat16_file_struct* handle;
char name[32];


static unsigned int systemTimeMs;
static unsigned int syncTime;
static unsigned char Timer0Flag;
static unsigned char Timer1Flag;
static unsigned char LogFlag;
static unsigned char WriteFlag;
static unsigned char DispFlag;
static unsigned char OdoFlag = TRUE;

static unsigned char logging = 0;               // to tell the interrupt if we are logging

unsigned int odometer_distance=0;
unsigned int distance_time = 0;

float speed = (float)0.0;
float diff_thresh = 2.5;

int32_t DY_OFFSET = 0x000;
int32_t WK_OFFSET = 0x2BC;
int32_t MO_OFFSET = 0x770;
int32_t YR_OFFSET = 0xB0C;

int32_t DY_SIZE = 0x0AF;
int32_t WK_SIZE = 0x12D;
int32_t MO_SIZE = 0x0E7;
int32_t YR_SIZE = 0x173;

int32_t _time_offsets[4] = {0x000,0x2BC,0x770,0xB0C};
int32_t _time_sizes[4] = {0x0AF,0x12D,0x0E7,0x173};
int32_t* _offsets_ ;
unsigned int _graph[55];

/* Function Declarations */
//======================================
void UNDEF_Routine(void) __attribute__ ((interrupt("UNDEF")));

void rtc0(void);
void init_rtc(void);
void set_time(int yr,int mo,int dy_mo,int dy_yr,int dy_wk,int hr,int min,int sec);

void boot_up(void);
void LogomaticV2PllFeed(void);
void log_string(char *buf);

void card_init(void);

void flash_it(int);

void error_mode(char error_number);

static void initTimer0(void (*pISR)(void));             //,void (*pCallback)(void)
unsigned int getSystemTime(void);

//======================================

int main (void)
{
  boot_up(); //Init LPC, IO pins
        
  flash_it(500);

  flash_it(500);

  flash_it(500);

  flash_it(500);

  int   i = 0;
        
  LogFlag = FALSE;
  //WriteFlag = FALSE;
  //DispFlag = FALSE;
        
  //RandomCircles(100);
  //delay_ms(1500);
        
  ClearScreen();
  //Circle(50, 50,10,0xFF);
  //BufferToScreen(0,0,OLED_END,OLED_END);
  //delay_ms(1000);
        
  InitMenu();
        
  while(1)
    {   
#ifdef USE_SLOW_GPIO
      int _read_val = ~(IOPIN0>>SWITCH_UP);
#else
      int _read_val = ~(FIO0PIN>>SWITCH_UP);
#endif
      _read_val &= 0x07;                        // to leave just pins 10,11,12 (switches)
      if (_read_val)
        {
          switch (_read_val)
            {
            case 0x01:
              MenuAction(SWITCH_UP);
              delay_ms(50);
              break;
            case 0x02:
              MenuAction(SWITCH_ENTER);
              delay_ms(200);
              break;
            case 0x04:
              MenuAction(SWITCH_DOWN);
              delay_ms(50);
              break;
            default:
              break;
            }
        }
    }

  return 0;
}


/*****************************************************************************
 *
 * Description:
 *    Actual timer ISR that is called whenever timer 0 generated an interrupt.
 *
 ****************************************************************************/
static void
timer0ISR(void)
{
  //update system time (in ms)
  systemTimeMs += 1;
        
  //call timer callback, if registered
  //if (pTimer0Callback != NULL) (*pTimer0Callback)();
        
  if (systemTimeMs%LOG_INTERVAL==0) LogFlag=TRUE;
        
  if (systemTimeMs%ODO_INTERVAL==0 && logging == 1)
    {
#ifdef USE_SLOW_GPIO
      int _read_state = !((IOPIN0>>ODOMETER_PIN)&0x01);
#else
      int _read_state = !((FIO0PIN>>ODOMETER_PIN)&0x01);
#endif
                
      if (_read_state && OdoFlag)                       // if the pin is low (switch active) and it was high before
        {
          odometer_distance++;                          // increment distance counter
          OdoFlag=FALSE;
          //speed = ((float)60.0/(float)(systemTimeMs - distance_time))*(float)1000.0;
          //distance_time = systemTimeMs;
        }
      if (!_read_state) OdoFlag=TRUE;           // if the pin is high, set the flag
    }
        
  //if (systemTimeMs%DISP_INTERVAL==0) DispFlag=TRUE;

  T0IR        = 0xff;        //reset all IRQ flags
  VICVectAddr = 0x00;        //dummy write to VIC to signal end of interrupt
}


/*****************************************************************************
 *
 * Description:
 *    Initialize Timer #0 (and VIC) for generating system tick interrupts.
 *
 * Params:
 *    [in] pISR      - function pointer to timer ISR.
 *    [in] pCallback - function pointer to timer callback function.
 *                     NULL if no callback function shall be registered.
 *
 ****************************************************************************/
static void
initTimer0(void (*pISR)(void))          //, void (*pCallback)()
{       
  Timer0Flag = FALSE;

  //initialize VIC for Timer0 interrupts
  VICIntSelect &= ~0x10;       //Timer0 interrupt is assigned to IRQ (not FIQ)
  VICVectCntl1  = 0x24;        //enable vector interrupt for timer0
  VICVectAddr1  = (unsigned int)pISR;  //register ISR address
  VICIntEnable  |= 0x10;        //enable timer0 interrupt       
        
  //reset system time
  systemTimeMs = 0;

  //initialize and start Timer0
  T0TCR = 0x00000000;                           //disable and reset Timer0
  T0PR  = 0x00000000;
  T0PC  = 0x00000000;                           //no prescale of clock
  T0MR0 = 60000;                                                // 1 ms interrupts on 60 MHz clk
  T0IR  = 0x000000ff;                           //reset all flags before enable IRQs
  T0MCR = 0x00000003;                           //reset counter and generate IRQ on MR0 match
  T0CCR = 0x00000000;
  T0TCR = 0x00000001;                           //start Timer0
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void LogomaticV2PllFeed(void)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
{
  PLLFEED=0xAA;
  PLLFEED=0x55;
} // LogomaticV2PllFeed

//Basic file init and stuff
void boot_up(void)
{               
#ifdef USE_SLOW_GPIO
  SCS = 2;
  IODIR0 |= (1<<STAT0)|(1<<STAT1);
  IOCLR0 = (1<<STAT0)|(1<<STAT1);
#else
  SCS = 3;
  FIO0DIR |= (1<<STAT0)|(1<<STAT1);
  FIO0CLR = (1<<STAT0)|(1<<STAT1);
#endif
        
  // Setting Multiplier and Divider values
  PLLCFG=0x25; // M = 6
  LogomaticV2PllFeed();

  // Enabling the PLL */
  PLLCON=0x1;
  LogomaticV2PllFeed();

#define PLOCK 0x400
  // Wait for the PLL to lock to set frequency
  while(!(PLLSTAT & PLOCK)) ;

  // Connect the PLL as the clock source
  PLLCON=0x3;
  LogomaticV2PllFeed();

  // Enabling MAM and setting number of clocks used for Flash memory fetch (4 cclks in this case)
  //MAMTIM=0x3; //VCOM?
  MAMCR=0x2;
  MAMTIM=0x4; //Original

  VPBDIV=1;             // enable interrupts
  //VICVectAddr    = 0x00000000;         //clear any pending/active interrupts
  initTimer0(timer0ISR);
        
  rprintf_devopen(putc_serial0);                                                                //Open up serial port 0 for debugging
  rprintf("\n\n\n\tPushTracker v2.2\n");
  rprintf("\tMax Mobility LLC.\n");
  rprintf("\tWilliam Emfinger\n\n\n");
        
  OLED_init();
#ifdef DEBUG
  rprintf("Screen initialized.\n");
#endif
        
  init_rtc();
  set_time(2010,7,26,38,143,12,10,30);
#ifdef DEBUG
  rprintf("RTC initialized\n");
#endif

  UnselectAccel();
  initSSP();
#ifdef DEBUG
  rprintf("SSP initialized.\n");
#endif

  initAccel();
#ifdef DEBUG
  rprintf("Sensor initialized.\n");
#endif

  card_init(); //Init SD card interface
#ifdef DEBUG
  rprintf("uSD Card initialized.\n");
#endif

#ifdef USE_SLOW_GPIO
  IOSET0 = (1<<BLUE)|(1<<USB_LED);
#else
  FIO0SET = (1<<BLUE)|(1<<USB_LED);
#endif
}

/*****************************************************************************
 *
 * Description:
 *    Returns the current number of ms from system start.
 *
 * Returns:
 *    unsigned int - number of milliseconds since system start.
 *
 ****************************************************************************/
unsigned int
getSystemTime(void)
{
  return systemTimeMs;
}


/*************************************************************/
void rtc0(void)
{
  ILR |= 1;     // Clear interrupt flag
  VICVectAddr = 0;     // Acknowledge Interrupt
  PCON = 1;     // IDLE mode
} 

void init_rtc(void)
{
  ILR = 3;     // Disable 32'768 interrupt
  CCR = 0x11;     // Clock enable + 32'767Hz quartz enable
  CIIR = 0x01;     // Interupt every second
  VICVectAddr2 = (unsigned long)rtc0;     // set interrupt vector in 1
  VICVectCntl2 = 0x0000002D;     // use it for RTC Interrupt
  VICIntEnable = 0x00002000;     // Enable RTC Interrupt
}    
   
void set_time(int yr,int mo,int dy_mo,int dy_yr,int dy_wk,int hr,int min,int sec)
{
  YEAR = yr;            //2006;     // Year
  MONTH = mo;           //5;     // Month
  DOM = dy_mo;          //23;     // Day of month
  DOY = dy_yr;          //38;     // Day of year
  DOW = dy_wk;          //143;     // Day of week
  HOUR = hr;            //23;     // Hours
  MIN = min;            //14;     // Minutes
  SEC = sec;            //30;     // Seconds
}



/*****************************************************************************
 *
 * Description:
 *    Graphs the Acceleration data (x,y,z) on the screen until enter is pressed
 *                      Up/Down control scale of graph
 *
 * Returns:
 *    void
 *
 ****************************************************************************/
void GraphAccelVals(void)
{
  int accel_graph_x[52];
  int accel_graph_y[52];
  int accel_graph_z[52];
  int scale = 300;
  int i;
  for (i=0;i<51;i++)
    {
      accel_graph_x[i]=(accelX()*64)/scale;
      accel_graph_y[i]=(accelY()*64)/scale;
      accel_graph_z[i]=(accelZ()*64)/scale;
      delay_ms(10);
    }
        
  //int _read_state = (~(IO_PORT_PIN>>SWITCH_ENTER))&0x01;
#ifdef USE_SLOW_GPIO
  int _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
#else
  int _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
#endif
  while (_read_state != 1)
    {
      ClearDispBuff();
      accel_graph_x[51]=(accelX()*64)/scale;
      accel_graph_y[51]=(accelY()*64)/scale;
      accel_graph_z[51]=(accelZ()*64)/scale;
      GraphAccel(accel_graph_x,52, 0x03);
      GraphAccel(accel_graph_y,52, 0x3C);
      GraphAccel(accel_graph_z,52, 0xE0);
      BufferToScreen(0,0,OLED_END,OLED_END);
        
      for (i=0;i<51;i++)
        {
          accel_graph_x[i]=accel_graph_x[i+1];
          accel_graph_y[i]=accel_graph_y[i+1];
          accel_graph_z[i]=accel_graph_z[i+1];
        }
                
      //int _read_val = ~(IO_PORT_PIN>>SWITCH_UP);
#ifdef USE_SLOW_GPIO
      int _read_val = ~(IOPIN0>>SWITCH_UP);
#else
      int _read_val = ~(FIO0PIN>>SWITCH_UP);
#endif
      _read_val &= 0x07;                        // to leave just pins 10,11,12 (switches)
      if (_read_val)
        {
          switch (_read_val)
            {
            case 0x01:
              scale += 50;
              if (scale>700) scale = 700;
              delay_ms(10);
              break;
            case 0x02:
              _read_state = 1;
              break;
            case 0x04:
              scale -= 50;
              if (scale<50) scale = 50;
              delay_ms(10);
              break;
            default:
              delay_ms(10);
              break;
            }
        }
    }
}


/*****************************************************************************
 *
 * Description:
 *    Logs: filtered data, Push Number, Push Times to SD card
 *
 * Returns:
 *    void
 *
 ****************************************************************************/
void LogData(void)
{       
#ifdef USE_SLOW_GPIO
  IOCLR0 = (1<<BLUE);
  IODIR0 &= ~(1<<ODOMETER_PIN);         // to ensure that the odometer pin is input
#else
  FIO0CLR = (1<<BLUE);
  FIO0DIR &= ~(1<<ODOMETER_PIN);                // to ensure that the odometer pin is input
#endif
  char str_data[128];
  int initialize=0;
        
  odometer_distance=0;
  OdoFlag = TRUE;
  distance_time = 0;
        
  speed = (float)0.0;
        
  logging = 1;
        
  float mean;
  int i;
  int PushFlag;
  float min_thresh = 5.0;
  float thresh = min_thresh;
  struct accel_struct log_struct;
        
  const float B[5] = {0.00370094327515829,0,-0.00740188655031657,0,0.00370094327515829};                                // matlab bandpass filter : butter(2,[1/200 1/8])
  const float A[5] = {1,-3.81639890322756,5.46866457896694,-3.48775379557443,0.835492729504173};
        
  float z1=0.0;
  float z2=0.0;
  float z3=0.0;
  float z4=0.0;
        
  struct PushTracker_struct PT_struct;
        
  PT_struct.Pushes = 0;
  PT_struct.min_mask = 0;
  PT_struct.max_mask = 0;
        
  for (i=0;i<VAL_BUFFER_SIZE;i++)
    {
      PT_struct.min_times[i]=0;
      PT_struct.min_vals[i]=0;
      PT_struct.max_times[i]=0;
      PT_struct.max_vals[i]=0;
    }   
        
  for (i=0;i<RAF_BUFFER_SIZE;i++)       
    {
      PT_struct.RAF_diff[i]=0;
      PT_struct.push_times[i]=0;
    }
        
  //Create the main log file
  int count = 0;
  sprintf(name,"LOG%02d.csv",count);
  while(root_file_exists(name))
    {
      count++;
      if(count == 250) 
        {
          rprintf("Too Many Logs!\n");
          //error_mode(4);
        }
      sprintf(name,"LOG%02d.csv",count);
    }
  handle = root_open_new(name);

  sd_raw_sync();
        
  log_string("%Time_ms,%Filt_Y,%PushTimes,%Odometer,%Ang_Spd\n");
  rprintf("Logging...\n");
        
  LogFlag=FALSE;
        
#ifdef USE_SLOW_GPIO
  int _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
#else
  int _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
#endif
  while (_read_state != 1)
    {
      if(LogFlag==TRUE)
        {
          if (initialize<RAW_BUFFER_SIZE)
            {
              LogFlag=FALSE;
                                
              PT_struct.raw_buffer[initialize] = accelY();
              PT_struct.time_buffer[initialize] = systemTimeMs;
                                
              PT_struct.filtered_buffer[initialize] = (B[0]*PT_struct.raw_buffer[initialize])+z1;
              z1 = (B[1]*PT_struct.raw_buffer[initialize])+z2-(A[1]*PT_struct.filtered_buffer[initialize]);
              z2 = (B[2]*PT_struct.raw_buffer[initialize])+z3-(A[2]*PT_struct.filtered_buffer[initialize]);
              z3 = (B[3]*PT_struct.raw_buffer[initialize])+z4-(A[3]*PT_struct.filtered_buffer[initialize]);
              z4 = (B[4]*PT_struct.raw_buffer[initialize])-(A[4]*PT_struct.filtered_buffer[initialize]);
                                
              sprintf(str_data, "%05d,%05f,0,%05d,%05f\n",PT_struct.time_buffer[initialize],PT_struct.filtered_buffer[initialize],odometer_distance,speed);
              log_string(str_data);
                                
#ifdef USE_SLOW_GPIO
              _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
#else
              _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
#endif
                                
              initialize++;
            }
          else
            {
              LogFlag=FALSE;
                                
              PT_struct = minima(PT_struct);
              PT_struct = maxima(PT_struct);
                                
              if ((PT_struct.max_mask&0x01) == 0)               // to see if there is a new max
                {
                  if ((PT_struct.min_mask&0x01)==0)
                    {
                      if (PT_struct.max_times[VAL_BUFFER_SIZE-1]>PT_struct.min_times[VAL_BUFFER_SIZE-1]&& (PT_struct.max_times[VAL_BUFFER_SIZE-1]-PT_struct.min_times[VAL_BUFFER_SIZE-1])<2000)         // && (PT_struct.max_times[VAL_BUFFER_SIZE-1]-PT_struct.min_times[VAL_BUFFER_SIZE-1])<1500
                        {
                          PT_struct.min_mask |= 0x01;
                          PT_struct.max_mask |= 0x01;
                          for (i=0;i<RAF_BUFFER_SIZE-1;i++) PT_struct.RAF_diff[i] = PT_struct.RAF_diff[i+1];
                          PT_struct.RAF_diff[RAF_BUFFER_SIZE-1] = PT_struct.max_vals[VAL_BUFFER_SIZE-1]-PT_struct.min_vals[VAL_BUFFER_SIZE-1];
                                                        
                          if (PT_struct.RAF_diff[RAF_BUFFER_SIZE-1]>thresh) 
                            {
                              if ((PT_struct.min_times[VAL_BUFFER_SIZE-1]-PT_struct.push_times[RAF_BUFFER_SIZE-1])>300)
                                {
                                  for (i=0;i<(RAF_BUFFER_SIZE-1);i++) PT_struct.push_times[i]=PT_struct.push_times[i+1];
                                  PT_struct.push_times[RAF_BUFFER_SIZE-1] = PT_struct.min_times[VAL_BUFFER_SIZE-1];
                                  PT_struct.Pushes++;
                                  PushFlag=TRUE;
                                }
                            }
                          mean=0.0;
                          i=0;
                          while (PT_struct.RAF_diff[i]) i++;
                          if (i==RAF_BUFFER_SIZE)
                            {
                              for (i=0;i<RAF_BUFFER_SIZE;i++)   mean+=PT_struct.RAF_diff[i];
                              mean = mean/RAF_BUFFER_SIZE;
                              thresh=mean*(float)1/2;
                              if (thresh<min_thresh)
                                {
                                  thresh=min_thresh;
                                }
                            }
                        }
                      else 
                        {
                          if (PT_struct.max_times[VAL_BUFFER_SIZE-1]>PT_struct.min_times[VAL_BUFFER_SIZE-2] && PT_struct.min_mask&0x02 == 0)
                            {
                              PT_struct.min_mask |= 0x02;
                              PT_struct.max_mask |= 0x01;
                              for (i=0;i<RAF_BUFFER_SIZE-1;i++) PT_struct.RAF_diff[i] = PT_struct.RAF_diff[i+1];
                              PT_struct.RAF_diff[RAF_BUFFER_SIZE-1] = PT_struct.max_vals[VAL_BUFFER_SIZE-1]-PT_struct.min_vals[VAL_BUFFER_SIZE-2];
                                                                
                              if (PT_struct.RAF_diff[RAF_BUFFER_SIZE-1]>thresh) 
                                {
                                  if ((PT_struct.min_times[VAL_BUFFER_SIZE-2]-PT_struct.push_times[RAF_BUFFER_SIZE-1])>300)
                                    {
                                      for (i=0;i<(RAF_BUFFER_SIZE-1);i++) PT_struct.push_times[i]=PT_struct.push_times[i+1];
                                      PT_struct.push_times[RAF_BUFFER_SIZE-1] = PT_struct.min_times[VAL_BUFFER_SIZE-2];
                                      PT_struct.Pushes++;
                                      PushFlag=TRUE;
                                    }
                                }
                              //mean=0.0;
                              //i=0;
                              //while (PT_struct.RAF_diff[i]) i++;
                              //if (i==RAF_BUFFER_SIZE)
                              //{
                              //        for (i=0;i<RAF_BUFFER_SIZE;i++) mean+=PT_struct.RAF_diff[i];
                              //        mean = mean/(float)RAF_BUFFER_SIZE;
                              //        thresh=mean*(float)1/2;
                              //        if (thresh<min_thresh)
                              //        {
                              //                thresh=min_thresh;
                              //        }
                              //}
                            }
                          else PT_struct.max_mask |= 0x01;
                        }
                    }
                }
                                
              for (i=0;i<RAW_BUFFER_SIZE-1;i++)
                {
                  PT_struct.time_buffer[i] = PT_struct.time_buffer[i+1];
                  PT_struct.raw_buffer[i] = PT_struct.raw_buffer[i+1];
                  PT_struct.filtered_buffer[i] = PT_struct.filtered_buffer[i+1];
                }
              PT_struct.raw_buffer[RAW_BUFFER_SIZE-1] = accelY();
              PT_struct.time_buffer[RAW_BUFFER_SIZE-1] = systemTimeMs;
                                
              PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1] = (B[0]*PT_struct.raw_buffer[RAW_BUFFER_SIZE-1])+z1;
              z1 = (B[1]*PT_struct.raw_buffer[RAW_BUFFER_SIZE-1])+z2-(A[1]*PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1]);
              z2 = (B[2]*PT_struct.raw_buffer[RAW_BUFFER_SIZE-1])+z3-(A[2]*PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1]);
              z3 = (B[3]*PT_struct.raw_buffer[RAW_BUFFER_SIZE-1])+z4-(A[3]*PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1]);
              z4 = (B[4]*PT_struct.raw_buffer[RAW_BUFFER_SIZE-1])-(A[4]*PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1]);
                                
              if (PushFlag==TRUE)
                { 
                  sprintf(str_data, "%05d,%05f,%05d,%05d,%05f\n",PT_struct.time_buffer[RAW_BUFFER_SIZE-1],PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1],PT_struct.push_times[RAF_BUFFER_SIZE-1],odometer_distance,speed); 
                  PushFlag=FALSE;
                }
              else sprintf(str_data, "%05d,%05f,0,%05d,%05f\n",PT_struct.time_buffer[RAW_BUFFER_SIZE-1],PT_struct.filtered_buffer[RAW_BUFFER_SIZE-1],odometer_distance,speed);
              log_string(str_data);
                                
#ifdef USE_SLOW_GPIO
              _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
#else
              _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
#endif
            }
        }
    }
        
  logging =0;
        
        
  sprintf(str_data, "Pushes=%05d\n",PT_struct.Pushes);
  log_string(str_data);
        
  /* Close the file! */
  sd_raw_sync();
  fat16_close_file(handle);
        
#ifdef USE_SLOW_GPIO
  IOSET0 = (1<<BLUE);
#else
  FIO0SET = (1<<BLUE);
#endif
}

// New minfinder functional generator!
// This looks for minima by seraching and indexing when you have a minima in
// the middle of a set of points, not at either end

struct PushTracker_struct minima(struct PushTracker_struct PT)
{ 
  int i;
  int _index = 0;
  for (i=1; i<=(RAW_BUFFER_SIZE-1); i++) _index = (PT.filtered_buffer[_index]<PT.filtered_buffer[i]) ? _index : i;
        
  if (_index>0 && _index<(RAW_BUFFER_SIZE-1))
    {
      if (PT.time_buffer[_index] > PT.min_times[VAL_BUFFER_SIZE-1])             //PT.time_buffer[_index] > PT.min_times[VAL_BUFFER_SIZE-1] && 
        {
          if (PT.max_times[VAL_BUFFER_SIZE-1] > PT.min_times[VAL_BUFFER_SIZE-1])
            {
              if (abs(PT.max_vals[VAL_BUFFER_SIZE-1]-PT.filtered_buffer[_index])>diff_thresh && (PT.time_buffer[_index]-PT.max_times[VAL_BUFFER_SIZE-1])>300)
                {
                  for (i=0;i<(VAL_BUFFER_SIZE-1);i++)                           //if (abs(PT.max_vals[VAL_BUFFER_SIZE-1]-PT.filtered_buffer[_index])>diff_thresh)               // to make sure that the min is different enough from the last ma
                    {
                      PT.min_times[i] = PT.min_times[i+1];
                      PT.min_vals[i] = PT.min_vals[i+1];
                    }
#ifdef DEBUG
                  rprintf("min=%05d\n",PT.time_buffer[_index]);
#endif
                  PT.min_mask = PT.min_mask << 1;
                  PT.min_times[VAL_BUFFER_SIZE-1] = PT.time_buffer[_index];
                  PT.min_vals[VAL_BUFFER_SIZE-1] = PT.filtered_buffer[_index];
                }
            }
          else
            {
              if (PT.min_vals[VAL_BUFFER_SIZE-1]<=PT.filtered_buffer[_index]) return PT;
#ifdef DEBUG
              rprintf("min=%05d\n",PT.time_buffer[_index]);
#endif
              PT.min_mask &= ~(0x01);                   // sets bit 0 to 0
              PT.min_times[VAL_BUFFER_SIZE-1] = PT.time_buffer[_index];
              PT.min_vals[VAL_BUFFER_SIZE-1] = PT.filtered_buffer[_index];
              //PT.max_mask &= ~(0x01);                 // to check the last min with the new max
            }
        }
    }
  return PT;
}

// New maxfinder functional generator!
// This looks for maxes by seraching and indexing when you have a mmaxima in
// the middle of a set of points, not at either end

struct PushTracker_struct maxima(struct PushTracker_struct PT)
{ 
  int i;
  int _index = 0;
  for (i=1; i<=(RAW_BUFFER_SIZE-1); i++) _index = (PT.filtered_buffer[_index]>PT.filtered_buffer[i]) ? _index : i; 
        
  if (_index>0 && _index<(RAW_BUFFER_SIZE-1))
    {
      if (PT.time_buffer[_index] > PT.max_times[VAL_BUFFER_SIZE-1])
        {
          if (PT.min_times[VAL_BUFFER_SIZE-1] > PT.max_times[VAL_BUFFER_SIZE-1])
            {
              if (abs(PT.min_vals[VAL_BUFFER_SIZE-1]-PT.filtered_buffer[_index])>diff_thresh && (PT.time_buffer[_index]-PT.min_times[VAL_BUFFER_SIZE-1])>300)
                {
                  for (i=0;i<(VAL_BUFFER_SIZE-1);i++)
                    {
                      PT.max_times[i] = PT.max_times[i+1];
                      PT.max_vals[i] = PT.max_vals[i+1];
                    }
#ifdef DEBUG
                  rprintf("max=%05d\n",PT.time_buffer[_index]);
#endif
                  PT.max_mask = PT.max_mask << 1;
                  PT.max_times[VAL_BUFFER_SIZE-1] = PT.time_buffer[_index];
                  PT.max_vals[VAL_BUFFER_SIZE-1] = PT.filtered_buffer[_index];
                }
            }
          else
            {
              if (PT.max_vals[VAL_BUFFER_SIZE-1]>=PT.filtered_buffer[_index]) return PT;
#ifdef DEBUG
              rprintf("max=%05d\n",PT.time_buffer[_index]);
#endif
              PT.max_mask &= ~(0x01);                   // sets bit 0 to 0
              PT.max_times[VAL_BUFFER_SIZE-1] = PT.time_buffer[_index];
              PT.max_vals[VAL_BUFFER_SIZE-1] = PT.filtered_buffer[_index];
              PT.min_mask &= ~(0x01);                   // to check the last min with the new max
            }
        }
    }
  return PT;
}


/*****************************************************************************
 *
 * Description:
 *    Loads the data from the SD card (data.csv) to graph to the screen.
 *
 * Returns:
 *    void
 *
 ****************************************************************************/
void load_data(int time, int variable)
{
#define READBUFSIZE 512

  /* readbuf MUST be on a word boundary */
  unsigned char readbuf[READBUFSIZE];
  char filename[12] = "data.csv";
        
  int local_index=0;

  int read;
  int i;

  if(root_file_exists(filename))
    {
      /* Open the file */
      handle = root_open(filename);
#ifdef DEBUG
      rprintf("Data file found,\nloading values into memory.\n");
      rprintf("time = %02d\n",time);
      rprintf("var = %02d\n",variable);
#endif
                
      int32_t _off_= _time_offsets[time] + variable*_time_sizes[time];
      _offsets_ =&_off_;
      if(!fat16_seek_file(handle, _offsets_, 0))        error_mode(10);         // FAT16_SEEK_SET=0 relative to beginning
                
      /* Clear the buffer */
      for(i=0;i<READBUFSIZE;i++) readbuf[i]=0;
                
      for (i=0;i<53;i++) _graph[i]=0;
                
      /* Read the file contents, and print them out */
      read=fat16_read_file(handle,(unsigned char*)readbuf,READBUFSIZE);
#ifdef DEBUG
      rprintf("bytes read = %05d\n",read);
#endif
      i=0;
      int readval = 0;
      while(readval!=0x0A)
        {                       
          if (readbuf[i]==',' || readbuf[i] == 0x0A)
            {
              int _ind=1;
              unsigned int value = 0;
                                
              //for(_ind=1;_ind<7;_ind++)               //because all numbers are 6 decimal digits
              while (readbuf[i-_ind]!=',' && readbuf[i-_ind]!=0x0A && (i-_ind)>=0)
                {
                  switch (_ind)
                    {
                    case 1:
                      value +=(readbuf[i-_ind]-0x30);
                      break;
                    case 2:
                      value +=(readbuf[i-_ind]-0x30)*10;
                      break;
                    case 3:
                      value +=(readbuf[i-_ind]-0x30)*100;
                      break;
                    case 4:
                      value +=(readbuf[i-_ind]-0x30)*1000;
                      break;
                    case 5:
                      value +=(readbuf[i-_ind]-0x30)*10000;
                      break;
                    case 6:
                      value +=(readbuf[i-_ind]-0x30)*100000;
                      break;
                    default:
                      break;
                    }
                  _ind++;
                }
                                
              //#ifdef DEBUG
              //rprintf("value,%03d\n",value);
              //#endif
                                
              _graph[local_index] = value;
                                
                                
#ifdef DEBUG
              rprintf("graph,%03d\n",_graph[local_index]);
#endif
                                
              local_index++;
            }
          readval = readbuf[i];
          i++;
        }
                
    }
        
  else
    {
      /* Create the file */
      handle = root_open_new(filename);
#ifdef DEBUG
      log_string("Maybe this will work?\nNope\n");
      rprintf("No data file present,\ncreating new data File.\n");
#endif
    }
  /* Close the file! */
  sd_raw_sync();
  fat16_close_file(handle);
}


void log_string(char *buf)
{
  int stringSize = strlen(buf);
  fat16_write_file(handle, (unsigned char*)buf, stringSize);
  sd_raw_sync();
}


//Turn on/off the two status LEDs simultaneously
void flash_it(int length_of_flash)
{
#ifdef USE_SLOW_GPIO
  for(int i = 0; i < 5; i++)
    {
      IOSET0 = (1<<STAT0);
      IOCLR0 = (1<<STAT1);
      delay_ms(length_of_flash);
                
      IOSET0 = (1<<STAT1);
      IOCLR0 = (1<<STAT0);
      delay_ms(length_of_flash);
    }
#else
  for(int i = 0; i < 5; i++)
    {
      FIO0SET = (1<<STAT0);
      FIO0CLR = (1<<STAT1);
      delay_ms(length_of_flash);
                
      FIO0SET = (1<<STAT1);
      FIO0CLR = (1<<STAT0);
      delay_ms(length_of_flash);
    }
#endif
}

void card_init(void)
{
  if(!sd_raw_init()) //Attempt to init SD raw mode
    {
      rprintf("SD Init Error\n\r");
      error_mode(2);
    }

  if(openroot()) //Attempt to open root directory
    { 
      rprintf("SD OpenRoot Error\n\r");
      error_mode(3);
    }
}

//Blinks stat LED in an error number
void error_mode(char error_number)
{
  error_number++;
        
  while(1)
    {
      for(char i = 0 ; i < error_number ; i++)
        {
#ifdef USE_SLOW_GPIO
          IOCLR0 = (1<<STAT0);
          delay_ms(250);
          IOSET0 = (1<<STAT0);
          delay_ms(250);
#else
          FIO0CLR = (1<<STAT0);
          delay_ms(250);
          FIO0SET = (1<<STAT0);
          delay_ms(250);
#endif
        }

      delay_ms(2000);
    }
}


void reset_system(void)
{
  // Intentionally fault Watchdog to trigger a reset condition
  WDMOD |= 3;
  WDFEED = 0xAA;
  WDFEED = 0x55;
  WDFEED = 0xAA;
  WDFEED = 0x00;
}

//Not really true, but close to a delay of a milisecond
void delay_ms(unsigned int count)
{
  int i;
  count *= 10000;
  for(i = 0; i < count; i++)
    asm volatile ("nop");
}

/**********************************************************
Delay in ms
**********************************************************/
//void delay_ms(unsigned int count)
//{
//      unsigned int _final_time;
//      _final_time = getSystemTime() + count;
//      while (_final_time > getSystemTime());          // delay until system time has reached final time
//}
