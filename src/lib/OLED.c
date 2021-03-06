
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "OLED.h"
#include "ADXL345.h"
#include "main.h"
//#include "string_printf.h"
#include "fonts.h"

/* OLED Commands */
static const enum
{
  SSD1339ColAddr = 0x15,	// 2 data bytes: start, end addr (range=0-131; def=0,131)
	
  SSD1339WriteRAM = 0x5c,	// enable MCU write to RAM
  SSD1339ReadRAM = 0x5d,	// enable MCU read from RAM
	
  SSD1339RowAddr = 0x75,	// 2 data bytes: start, end addr (range=0-131; def=0,131)
	
  SSD1339DrawLine = 0x83,	// 6: colStart, rowStart, colEnd, rowEnd, color1, color2 (start < end < 132)
  SSD1339DrawRect = 0x84,	// 8: colStart, rowStart, colEnd, rowEnd, line1, line2, fill1, fill2 (start < end < 132)
  SSD1339DrawCirc = 0x86,	// 7: colCenter, rowCenter, radius, line1, line2, fill1, fill2
  SSD1339Copy = 0x8a,		// 6: colStart, rowStart, colEnd, rowEnd, colNew, rowNew (start < end < 132)
  SSD1339DimWin = 0x8c,	// 4: colStart, rowStart, colEnd, rowEnd (start < end < 132)
  SSD1339ClearWin = 0x8e,	// 4: colStart, rowStart, colEnd, rowEnd (start < end < 132)
	
  SSD1339DrawSettings = 0x92,
  SSD1339HScroll = 0x96,
  SSD1339StartScroll = 0x9f,
  SSD1339StopScroll = 0x9e,
	
  SSD1339Settings = 0xa0,	// 1 data byte (see doc) [bit3=1 for 9-bit bus] [bit7:6=01 for 65k color (def't)]
  SSD1339StartLine = 0xa1,	// 1 data byte: vertical scroll by RAM (range=0-131; def=0)
  SSD1339Offset = 0xa2,	// 1 data byte: vertical scroll by Row (range=0-131; def=0)
  SSD1339DispAllOff = 0xa4,
  SSD1339DispAllOn = 0xa5,		// all pixels have GS15
  SSD1339DispNormal = 0xa6,	// default
  SSD1339DispInverse = 0xa7,	// GS0->GS63, GS1->GS62, ...
  SSD1339Config = 0xad,	// 1 data byte (see doc) [1000 1_1_ => 1000 1110 = 0x8e]

  SSD1339SleepOn = 0xae,		// (display off)
  SSD1339SleepOff = 0xaf,		// (display on)

  SSD1339PowerSaving = 0xb0,	// 1 data byte (see doc) [00: internal VSL, normal (def't); 0x12: power saving internal VSL]
  SSD1339SetPeriod = 0xb1,	// 1 data byte: low nibble: phase 1 (reset) period [4]; high nibble: phase 2 [7](pre-charge) period; 1-16DCLK
  SSD1339Clock = 0xb3,		// 1 data byte: low nibble: divide by DIVSET+1 [0]; high nibble: osc freq [9? d?]
  SSD1339GrayPulseWidth = 0xb8,	// 32 bytes (see doc)
  SSD1339LinearLUT = 0xb9,		// reset to default (linear grayscale) Look Up Table (PW1=1, PW2=3, PW3=GRAPH_LEFT ... PW63=125)
  SSD1339PrechargeV = 0xbb,	// 3 bytes: colors A, B, C; def't=0x1c; 0: 0.51*Vref, 1f: 0.84*Vref, 0x80: Vcomh
  SSD1339SetVcomh = 0xbe,		// 1 byte: 0: 0.51*Vref, 1f: 0.84*Vref (def't)

  SSD1339ColorContrast = 0xc1,	// 3 bytes: colors A, B, C; def't=0x80 (sets current)
  SSD1339Contrast = 0xc7,		// 1 byte: reduce output current (all colors) to (low nibble+1)/16 [def't=0f]
  SSD1339MUXRatio = 0xca,	// 1 byte: 16MUX-132MUX (range=15-131; def=131)

  SSD1339NOP = 0xe3,

  SSD1339Lock = 0xfd,		// 1 byte: 0x12=unlock (def't); 0x16=lock


  //---------------------------------------------------
  // For the updated screen with SSD1351U driver
	
  SSD1351ColAddr = 0x15,	// 2 data bytes: start, end addr (range=0-131; def=0,131)
	
  SSD1351WriteRAM = 0x5c,	// enable MCU write to RAM
  SSD1351ReadRAM = 0x5d,	// enable MCU read from RAM
	
  SSD1351RowAddr = 0x75,	// 2 data bytes: start, end addr (range=0-131; def=0,131)

  SSD1351HScroll = 0x96,
  SSD1351StartScroll = 0x9f,
  SSD1351StopScroll = 0x9e,

  SSD1351Settings = 0xa0,	// 1 data byte (see doc) [bit3=reserved] [bit7:6=01 for 65k color (def't)]	
  SSD1351StartLine = 0xa1,	// 1 data byte: vertical scroll by RAM (range=0-131; def=0)

  SSD1351Offset = 0xa2,	// 1 data byte: vertical scroll by Row (range=0-131; def=0)
  SSD1351DispAllOff = 0xa4,
  SSD1351DispAllOn = 0xa5,		// all pixels have GS15
  SSD1351DispNormal = 0xa6,	// default
  SSD1351DispInverse = 0xa7,	// GS0->GS63, GS1->GS62, ...
  SSD1351Config = 0xAB,	
	
  SSD1351NOP = 0xAD,
  SSD1351SleepOn = 0xae,		// (display off)
  SSD1351SleepOff = 0xaf,		// (display on)

  SSD1351SetPeriod = 0xb1,	// 1 data byte: low nibble: phase 1 (reset) period [4]; high nibble: phase 2 [7](pre-charge) period; 1-16DCLK

  SSD1351SetPerformance = 0xb2,	// 3 data bytes 0,0,0 = normal (def't), A4h,0,0=enhance performance
  SSD1351Clock = 0xb3,		// 1 data byte: low nibble: divide by DIVSET+1 [0]; high nibble: osc freq [9? d?]

  SSD1351VSL = 0xB4,			// 3 data bytes:  A = 0b101000XX,  A1:A0 => 00: External VSL, 10: Internal VSL; B = 0b10110101; C = 0b01010101
	
  SSD1351GPIO = 0xB5, 		// A1:0 => GPIO0, A3:2 => GPIO1
  SSD1351SecondPeriod = 0xB6,	// A3:A0 => 0:invalid, 1: 1 DCLKS, 2: 2 DCLKS, 3: 3 Dclks,...15: 15 DCLKS
  SSD1351GrayPulseWidth = 0xb8,	// 63 bytes (see doc)
  SSD1351LinearLUT = 0xb9,		// reset to default (linear grayscale) Look Up Table (PW1=1, PW2=3, PW3=GRAPH_LEFT ... PW63=125)
  SSD1351PrechargeV = 0xbb,	// 1 byte: 0: 0.2*Vref, 1f: 0.6*Vcc, high 3 bits = 0  

  SSD1351SetVcomh = 0xbe,		// 1 byte: 0: 0.72*Vref, 1f: 0.84*Vref (def't)  A2:0,   rest are 0
	
  SSD1351ColorContrast = 0xc1,	// 3 bytes: colors A, B, C; def't=0x80 (sets current)
  SSD1351Contrast = 0xc7,		// 1 byte: reduce output current (all colors) to (low nibble+1)/16 [def't=0f]
  SSD1351MUXRatio = 0xca,	// 1 byte: 16MUX-128MUX (range=15-OLED_END; def=OLED_END)
	
  SSD1351Lock = 0xfd,		// 1 byte: 0x12=unlock (def't); 0x16=lock
};

unsigned char disp_buff[128][128];
unsigned char text_buff[60][127];
static const unsigned short color_matrix[256]= {
  0,8,23,31,256,264,279,287,512,520,535,543,768,776,791,799,1248,1256,1271,1279,1504,
  1512,1527,1535,1760,1768,1783,1791,2016,2024,2039,2047,8192,8200,8215,8223,8448,8456,
  8471,8479,8704,8712,8727,8735,8960,8968,8983,8991,9440,9448,9463,9471,9696,9704,9719,
  9727,9952,9960,9975,9983,10208,10216,10231,10239,16384,16392,16407,16415,16640,16648,
  16663,16671,16896,16904,16919,16927,17152,17160,17175,17183,17632,17640,17655,17663,
  17888,17896,17911,17919,18144,18152,18167,18175,18400,18408,18423,18431,24576,24584,
  24599,24607,24832,24840,24855,24863,25088,25096,25111,25119,25344,25352,25367,25375,
  25824,25832,25847,25855,26080,26088,26103,26111,26336,26344,26359,26367,26592,26600,
  26615,26623,38912,38920,38935,38943,39168,39176,39191,39199,39424,39432,39447,39455,
  39680,39688,39703,39711,40160,40168,40183,40191,40416,40424,40439,40447,40672,40680,
  40695,40703,40928,40936,40951,40959,47104,47112,47127,47135,47360,47368,47383,47391,
  47616,47624,47639,47647,47872,47880,47895,47903,48352,48360,48375,48383,48608,48616,
  48631,48639,48864,48872,48887,48895,49120,49128,49143,49151,55296,55304,55319,55327,
  55552,55560,55575,55583,55808,55816,55831,55839,56064,56072,56087,56095,56544,56552,
  56567,56575,56800,56808,56823,56831,57056,57064,57079,57087,57312,57320,57335,57343,
  63488,63496,63511,63519,63744,63752,63767,63775,64000,64008,64023,64031,64256,64264,
  64279,64287,64736,64744,64759,64767,64992,65000,65015,65023,65248,65256,65271,65279,
  65504,65512,65527,65535
};

static const char *var_menu[MAX_VARIABLES] = {"Cadence","Distance","Pushes","Speed","Reset","Pong","Graph","Log","Clock","Off"};
char *val_disp[MAX_VARIABLES] = {"56","2.0","50","3.0","  ","  ","  ","  ","  "," "};
static const char *time_scale[4][4] ={{"0","6a","12","6p"},{"0","M","W","F"},{"1-6","8-6","16-6","24-6"},{"Jan","Apr","Jul","Oct"}}; 
static const char time_divisions[4] = {DAY_SIZE,WEEK_SIZE,MONTH_SIZE,YEAR_SIZE};

static const char _row1 = 3;
static const char _row2 = 3+11+TEXT_SPACING;
static const char _row3 = 3+2*(11+TEXT_SPACING);
static const char _row4 = 3+3*(11+TEXT_SPACING);
unsigned char _current_row = 0;

unsigned char var_selection = 2;
unsigned char time_selection = 0;

/*****************************************************************************
 *
 * Description:
 *    Initializes the OLED screen
 *
 * Returns:
 *    
 *
 ****************************************************************************/
void OLED_init(void)
{
  //init_fonts();
  ClearText();
  UpdateText(_row1,0,0xFF);
  UpdateText(_row2,1,0xFF);
  UpdateText(_row3,2,0xFF);
  UpdateText(_row4,3,0xFF);
	
  unsigned long _dummy;
  _dummy = PINSEL2;
  _dummy &= ~((1<<2)|(1<<3));
  PINSEL2 = _dummy;	
	
#ifdef SSD1339
  FIO1DIR |= LCD_DATA|LCD_DC|LCD_RW|LCD_CS|LCD_RD|LCD_RSTB|BS1|BS2;
	
#ifdef USE_SLOW_GPIO
  IODIR0 |= OLED_PWR;
#else
  FIO0DIR |= OLED_PWR;
#endif
	
  FIO1CLR = BS1;
  FIO1SET = BS2;
	
  OLED_ON;
	
  delay_ms(500);
	
  FIO1CLR = LCD_RD|LCD_CS|LCD_RW;

  delay_ms(20);
  FIO1CLR = LCD_RSTB;
  delay_ms(200);
  FIO1SET = LCD_RSTB;
  delay_ms(20);

  write_c(SSD1339Settings); // Set Re-map / Color Depth
  write_d(0x34);//0xb4); // 262K 8bit R->G->B
  write_c(SSD1339StartLine); // Set display start line
  write_d(0x00); // 00h start
  //write_c(0xa2); // Set display offset
  //write_d(0x80); // 80h start
  write_c(SSD1339DispNormal); // Normal display
  write_c(SSD1339Config); // Setz Master Configuration
  write_d(0x8A); // DC-DC off & external VcomH voltage & external pre-charge voltage  // was 8e
  write_c(SSD1339PowerSaving); // Power saving mode
  write_d(0x05);
  write_c(SSD1339SetPeriod); // Set pre & dis_charge
  write_d(0x11); // pre=1h dis=1h
  write_c(SSD1339Clock); // clock & frequency
  write_d(0xf0); // clock=Divser+1 frequency=fh
  //write_c(SSD1339LinearLUT);
  write_c(SSD1339PrechargeV); // Set pre-charge voltage of color A B C
  write_d(0x1c); // color A
  write_d(0x1c); // color B
  write_d(0x1c); // color C
  write_c(SSD1339SetVcomh); // Set VcomH
  write_d(0x1f); //
  write_c(SSD1339ColorContrast); // Set contrast current for A B C
  write_d(0xC0); // Color A	
  write_d(0xC0); // Color B
  write_d(0xC0); // Color C
  write_c(SSD1339Contrast); // Set master contrast
  write_d(0x0f); // no change	
  write_c(SSD1339MUXRatio); // Duty
  write_d(127); // OLED_END+1
  write_c(SSD1339SleepOff); // Display on
  write_c(SSD1339DrawSettings);    // fill enable command
  write_d(0x01); 
  write_c(SSD1339ClearWin);
  write_d(0);
  write_d(0);
  write_d(127);
  write_d(127);
  delay_ms(5);
#else

  FIO1DIR |= LCD_DATA|LCD_DC|LCD_RW|LCD_CS|LCD_RD|LCD_RSTB|BS0|BS1;
#ifdef USE_SLOW_GPIO
  IODIR0 |= OLED_PWR;
#else
  FIO0DIR |= OLED_PWR;
#endif
  FIO1SET = BS1|BS0;
	
  OLED_OFF;
  delay_ms(500);
	
  FIO1CLR = LCD_RD|LCD_CS|LCD_RW;

  delay_ms(20);
  FIO1CLR = LCD_RSTB;
  delay_ms(200);
  FIO1SET = LCD_RSTB;
  delay_ms(20);
  write_c(SSD1351Lock);
  write_d(0x12);
  write_c(SSD1351Lock);
  write_d(0xB1);
  write_c(SSD1351SleepOn);
  write_c(SSD1351Clock); // clock & frequency
  write_d(0xF1); // clock=Divser+1 frequency=fh
  //ClearScreen();
  //BufferToScreen(0,0,127,127);
  write_c(SSD1351MUXRatio); // Duty
  write_d(127); // OLED_END+1
  write_c(SSD1351Settings); // Set Re-map / Color Depth
  write_d(0x34);//0xb4); 34		// 26 turns it upside down
  write_c(SSD1351ColAddr);
  write_d(0);
  write_d(127);
  write_c(SSD1351RowAddr);
  write_d(0);
  write_d(127);
  write_c(SSD1351StartLine); // Set display start line
  write_d(0x00); // 00h start
  write_c(SSD1351Offset); // Set display offset
  write_d(0x00); // 80h start
  write_c(SSD1351GPIO);
  write_d(0x00);
  write_c(SSD1351Config); // Set Master Configuration
  write_d(0x01); // internal VDD
  write_c(SSD1351SetPeriod); // Set pre & dis_charge
  write_d(0x32); 		// 32
  write_c(SSD1351VSL);
  write_d(0xA0);		// A1:A0 => 00: External VSL, 10: Internal VSL
  write_d(0xB5);
  write_d(0x55);
  write_c(SSD1351PrechargeV); // Set pre-charge voltage of color A B C
  write_d(0x17);		//	-- reset = 0x17
  write_c(SSD1351SetVcomh); // Set VcomH
  write_d(0x05); // reset value 0x05
  write_c(SSD1351ColorContrast); // Set contrast current for A B C
  write_d(0xC8); // Color A	-- default
  write_d(0x80); // Color B
  write_d(0xC8); // Color C
  write_c(SSD1351SetPerformance);
  write_d(0xA4);
  write_d(0x00);
  write_d(0x00);
  write_c(SSD1351Contrast); // Set master contrast
  write_d(0x0f); // no change		-- was 0x0f
  write_c(SSD1351SecondPeriod);
  write_d(0x01);
  write_c(SSD1351DispNormal); // Normal display
  ClearScreen();
  BufferToScreen(0,0,127,127);
  OLED_ON;
  write_c(SSD1351SleepOff); // Display on
#endif
}

void OLED_ShutDown(void)
{
  write_c(SSD1351SleepOn);
  delay_ms(100);
  OLED_OFF;
  delay_ms(500);
}

void OLED_TurnOn(void)
{
  OLED_ON;
  delay_ms(500);
  write_c(SSD1351SleepOff);
  delay_ms(100);
}

void ClearScreen(void)
{
  int row,col;
  for (row = 0; row<=OLED_END;row++)
    {
      for (col = 0;col<=OLED_END;col++) disp_buff[row][col] = 0;
    }
}

void ClearWindow(unsigned char x1,unsigned char y1, unsigned char x2, unsigned char y2)
{
  int row,col;
  for (row = y1; row<=y2;row++)
    {
      for (col = x1;col<=x2;col++) disp_buff[row][col] = 0;
    }
}

//void ScreenToBuffer(unsigned char x_start,unsigned char y_start,unsigned char x_end, unsigned char y_end)
//{
//	write_c(0x15);	// set column start and end addresses
//	write_d(x_start);
//	write_d(x_end);
//	write_c(0x75);	// set row start and end adresses
//	write_d(y_start);
//	write_d(y_end);
//	write_c(0x5D);	// command to read from Display DRAM
//	read_d();		// dummy read (as per datasheet)
//	int row,col;
//	for (row=y_start;row<=y_end;row++)
//	{
//		for (col=x_start;col<=x_end;col++)
//		{
//			disp_buff[row][col] = read_d();
//		}
//	}
//}

void BufferToScreen(unsigned char x_start,unsigned char y_start,unsigned char x_end,unsigned char y_end)
{
  write_c(0x15);	// set column start and end addresses
  write_d(x_start);
  write_d(x_end);
  write_c(0x75);	// set row start and end adresses
  write_d(y_start);
  write_d(y_end);
  write_c(0x5C);	// write to RAM command
  int row,col;
  for (row=y_start;row<=y_end;row++)
    {
      for (col=x_start;col<=x_end;col++)
	{
#ifdef SSD1339
	  write_d(disp_buff[row][col]);
#else
	  write_color(color_matrix[disp_buff[row][col]]);
#endif
	}
    }
}

//==============================================================================================
//=================================  Text Functions  ===========================================
//==============================================================================================

void Draw_5x8_char(char* _char_matrix,int x_start,int y_start,unsigned char clr)
{
  int row,col;
  for (col=0;col<=4;col++)
    {
      for (row=0;row<=7;row++)
	{
	  if (!((_char_matrix[col]>>row)&0x01)) 
	    {
	      if ((row+y_start)>=0 && (row+y_start)<=127 && (col+x_start)>=0 && (col+x_start)<=127) disp_buff[row+y_start][col+x_start] = clr;
	    }
	}
    }
}

void Draw_5x8_string(char* str,unsigned char len,int x_start,int y_start,unsigned char clr)
{
  int i = 0;
  for (i=0;i<len;i++) Draw_5x8_char(char5x8_matrix[str[i]],x_start+i*6,y_start,clr);
}

void Draw_8x12_char(char* _char_matrix,int x_start,int y_start,unsigned char clr)
{
  int row;
  int col;
  for (row=0;row<12;row++)
    {
      for (col=0;col<8;col++)
	{		
	  if (((_char_matrix[row]>>(7-col))&0x01)) 
	    {
	      if ((row+y_start)>=0 && (row+y_start)<=127 && (col+x_start)>=0 && (col+x_start)<=127)disp_buff[row+y_start][col+x_start] = clr;
	    }
	}
    }
}

void Write_8x12_char(char* _char_matrix,int x_start,int y_start,unsigned char clr)
{
  int row;
  int col;
  for (row=0;row<12;row++)
    {
      for (col=0;col<8;col++)
	{		
	  if (((_char_matrix[row]>>(7-col))&0x01)) 
	    {
	      text_buff[row+y_start][col+x_start] = clr;
	    }
	}
    }
}

void Draw_8x12_string(char* str,unsigned char len,int x_start,int y_start,unsigned char clr)
{
  int i = 0;
  for (i=0;i<len;i++) 
    {
      Draw_8x12_char(char8x12_matrix[str[i]],x_start+i*9,y_start,clr);
    }
}

void Write_8x12_string(char* str,unsigned char len,int x_start,int y_start,unsigned char clr)
{
  int i = 0;
  for (i=0;i<len;i++) 
    {
      Write_8x12_char(char8x12_matrix[str[i]],x_start+i*9,y_start,clr);
    }
}

//==============================================================================================
//=================================  Menu Functions  ===========================================
//==============================================================================================

void ClearGraph(void)
{
  int row,col;
  for (row = GRAPH_START; row<=GRAPH_END;row++)
    {
      for (col = 0;col<=OLED_END;col++) disp_buff[row][col] = 0;
    }
}

void ClearTimes(void)
{
  int row,col;
  for (row = TIME_HEIGHT; row<=OLED_END;row++)
    {
      for (col = 0;col<=OLED_END;col++) disp_buff[row][col] = 0;
    }
}

unsigned char ScrollVariables(unsigned char _dir,unsigned char _spd)
{
  ClearWindow(0,0,127,35);
  if (_dir==DOWN) 
    {
      _current_row = 0;
      unsigned char _new_row = 15;
		
      unsigned char row_string = var_selection - 1;
      if (row_string >(MAX_VARIABLES-1)) row_string = (MAX_VARIABLES-1);

      UpdateText(_row2,row_string,0xFF);
		
      row_string = row_string - 1;
      if (row_string >(MAX_VARIABLES-1)) row_string = (MAX_VARIABLES-1);
		
      UpdateText(_row1,row_string,0xFF);
		
      var_selection = (var_selection+1)%MAX_VARIABLES;
		
      row_string = var_selection - 1;
      if (row_string >(MAX_VARIABLES-1)) row_string = (MAX_VARIABLES-1);
		
      UpdateText(_row3,row_string,0xFF);
      UpdateText(_row4,var_selection,0xFF);
		
      while (_current_row<_new_row || (_new_row < TEXT_HEIGHT && _current_row > _new_row))
	{
	  _current_row = (_current_row+_spd)%60;
	  DrawVariables();
	}
      while (_current_row!=_new_row)
	{
	  _current_row--;
	  DrawVariables();
	}
    }
  else 
    {
      _current_row = 15;
      unsigned char _new_row = 0;
		
      var_selection--;
      if (var_selection >(MAX_VARIABLES-1)) var_selection = (MAX_VARIABLES-1);
		
      unsigned char row_string = var_selection - 1;
      if (row_string >(MAX_VARIABLES-1)) row_string = (MAX_VARIABLES-1);
		
      UpdateText(_row2,row_string,0xFF);
      UpdateText(_row3,var_selection,0xFF);
      UpdateText(_row4,(var_selection+1)%(MAX_VARIABLES),0xFF);
		
      row_string--;
      if (row_string >(MAX_VARIABLES-1)) row_string = (MAX_VARIABLES-1);
		
      UpdateText(_row1,row_string,0xFF);
		
      while (_current_row>_new_row || (_new_row > _current_row && _new_row > (60 - TEXT_HEIGHT)) )
	{
	  _current_row-=_spd;
	  if (_current_row>60) _current_row = 60;
	  DrawVariables();
	}
      while (_current_row!=_new_row)
	{
	  _current_row++;
	  DrawVariables();
	}
    }
  return var_selection;
}

unsigned char ScrollTimes(unsigned char _dir)
{
  if (_dir==DOWN) time_selection = (time_selection+1)%(NUM_MEASUREMENTS);
  else 
    {
      time_selection--;
      if (time_selection >(NUM_MEASUREMENTS-1)) time_selection = (NUM_MEASUREMENTS-1);
    }
  return time_selection;
}

void DrawTimes(unsigned char clr)
{
  int i;
  ClearTimes();
  for (i=0;i<4;i++)
    {
      if (var_selection<NUM_MEASUREMENTS)
	{
	  if (time_selection==0 && i!=0) Draw_5x8_string(time_scale[time_selection][i],strlen(time_scale[time_selection][i]),((GRAPH_RIGHT-GRAPH_LEFT)*i)/4,TIME_HEIGHT,clr);
	  if (time_selection==1 && i!=0) Draw_5x8_string(time_scale[time_selection][i],strlen(time_scale[time_selection][i]),((GRAPH_RIGHT-GRAPH_LEFT)*(2*i-1))/6,TIME_HEIGHT,clr);
	  if (time_selection==2 && (i==1||i==3)) Draw_5x8_string(time_scale[time_selection][i],strlen(time_scale[time_selection][i]),((GRAPH_RIGHT-GRAPH_LEFT)*(i))/4,TIME_HEIGHT,clr);
	  if (time_selection==3) Draw_5x8_string(time_scale[time_selection][i],strlen(time_scale[time_selection][i]),((GRAPH_RIGHT-GRAPH_LEFT)*(i))/4,TIME_HEIGHT,clr);
	}
    }
}

void ClearText(void)
{
  int i;
  int j;
  for (i=0;i<127;i++)
    {
      for (j=0;j<32;j++) text_buff[j][i] = 0;
    }
}

void UpdateText(unsigned char text_row,unsigned char text_index,unsigned char clr)
{
  int i;
  int j;
  char val_str[6];
  for (i=0;i<=127;i++)
    {
      for (j=text_row;j<text_row+12;j++) text_buff[j][i] = 0;
    }
	
  if (text_index == CLOCK_INDEX)
    {
      sprintf(val_str,"%02d:%02d",(int)HOUR,(int)MIN);
      val_disp[text_index] = val_str;
    }
	
  if (text_index == var_selection) 
    {
      Write_8x12_string(var_menu[text_index],strlen(var_menu[text_index]),TEXT_OFFSET,text_row,clr);
      Write_8x12_string(val_disp[text_index],strlen(val_disp[text_index]),OLED_END - strlen(val_disp[text_index])*9,text_row,clr);
    }
  else
    {
      Write_8x12_string(var_menu[text_index],strlen(var_menu[text_index]),TEXT_OFFSET,text_row,clr&(0xB6));
      Write_8x12_string(val_disp[text_index],strlen(val_disp[text_index]),OLED_END - strlen(val_disp[text_index])*9,text_row,clr&(0xB6));
    }
}

void DrawVariables(void)
{	
  int row,col;
  for (row=0;row<=TEXT_HEIGHT;row++)
    {
      for (col=0;col<=127;col++)
	{
	  disp_buff[row][col] = (text_buff[(_current_row+row)%60][col]);
	}
    }
  BufferToScreen(0,0,127,TEXT_HEIGHT);
}

//==============================================================================================
//=============================  Graphing Functions  ===========================================
//==============================================================================================


// 'cx' and 'cy' denote the offset of the circle centre from the origin.
void circle(int cx, int cy, int radius, unsigned char clroutline, unsigned char clrfill)
{
  int error = -radius;
  int x = radius;
  int y = 0;

  while (x > y)
    {
      plot8points(cx, cy, x, y,clroutline,clrfill);
 
      error += y;
      ++y;
      error += y;
 
      if (error >= 0)
	{
	  --x;
	  error -= x;
	  error -= x;
	}
    }
  plot4points(cx, cy, x, y,clroutline,clrfill);  
}
 
void plot8points(int cx, int cy, int x, int y, unsigned char clroutline,unsigned char clrfill)
{
  plot4points(cx, cy, x, y,clroutline,clrfill);
  plot4points(cx, cy, y, x,clroutline,clrfill);
}

void plot4points(int cx, int cy, int x, int y, unsigned char clroutline,unsigned char clrfill)
{
  int row,col;
  for (row = cy-y;row<=cy+y;row++)
    {
      for (col=cx-x;col<=cx+x;col++) 
	{
	  if (row>=0 && row<=127 && col>=0 && col<=127) disp_buff[row][col] = clrfill;
	}
    }
  if ((cy+y)>=0 && (cy+y)<=127 && (cx+x)>=0 && (cx+x)<=127) disp_buff[cy+y][cx+x] = clroutline;
  if (x != 0) 
    {
      if ((cy+y)>=0 && (cy+y)<=127 && (cx-x)>=0 && (cx-x)<=127)  disp_buff[cy+y][cx-x] = clroutline;
    }
  if (y != 0) 
    {
      if ((cy-y)>=0 && (cy-y)<=127 && (cx+x)>=0 && (cx+x)<=127)  disp_buff[cy-y][cx+x] = clroutline;
    }
  if (x != 0 && y != 0) 
    {
      if ((cy-y)>=0 && (cy-y)<=127 && (cx-x)>=0 && (cx-x)<=127)  disp_buff[cy-y][cx-x] = clroutline;
    }
}

void DrawRectangle(unsigned char xLeft, unsigned char yTop, unsigned char xRight, unsigned char yBottom, unsigned char clrOutline, unsigned char clrFill)
{	
  int row;
  int col;
  for (row=yTop;row<=yBottom;row++)
    {
      for (col=xLeft;col<=xRight;col++)
	{	
	  if (row>=0 && col>=0 && row<=127 && col<=127)
	    {
	      if (((col-xLeft)<2)||((xRight-col)<2)||((row-yTop)<2)||((yBottom-row)<2)) disp_buff[row][col] = clrOutline;
	      else disp_buff[row][col] = clrFill;
	    }	
	}
    }
}

void DrawLine(int xLeft, int yTop, int xRight, int yBottom, unsigned char clrFill)
{	
  int _dummy;
  int steep = (abs(yBottom - yTop) > abs(xRight - xLeft));
  if (steep)
    {
      _dummy = xLeft;
      xLeft = yTop;
      yTop = _dummy;
      _dummy = xRight;
      xRight = yBottom;
      yBottom = _dummy;
    }
  if (xLeft>xRight)
    {
      _dummy = xLeft;
      xLeft = xRight;
      xRight = _dummy;
      _dummy = yTop;
      yTop = yBottom;
      yBottom = _dummy;
    }
  int dx = xRight - xLeft;
  int dy = abs(yBottom - yTop);
  int error = dx>>1;		// divide by 2
  int ystep;
  int row = yTop;
  if (yTop<yBottom) ystep = 1;
  else ystep = -1;
	
  int col;
	
  for (col = xLeft;col <= xRight;col++)
    {
      if (row>=0 && col>=0 && row<=127 && col<=127)
	{
	  if (steep)	disp_buff[col][row] = clrFill;
	  else disp_buff[row][col] = clrFill;	
	}
      error = error - dy;
      if (error<0)
	{
	  row = row + ystep;
	  error = error + dx;
	}
    }
}

void DrawLineFill(int xLeft, int yTop, int xRight, int yBottom, unsigned char clrLine, unsigned char clrFill)
{	
  int _dummy;
  int steep = (abs(yBottom - yTop) > abs(xRight - xLeft));
  if (steep)
    {
      _dummy = xLeft;
      xLeft = yTop;
      yTop = _dummy;
      _dummy = xRight;
      xRight = yBottom;
      yBottom = _dummy;
    }
  if (xLeft>xRight)
    {
      _dummy = xLeft;
      xLeft = xRight;
      xRight = _dummy;
      _dummy = yTop;
      yTop = yBottom;
      yBottom = _dummy;
    }
  int dx = xRight - xLeft;
  int dy = abs(yBottom - yTop);
  int error = dx>>1;		// divide by 2
  int ystep;
  int row = yTop;
  if (yTop<yBottom) ystep = 1;
  else ystep = -1;
	
  int col;
  int i;
	
  for (col = xLeft;col <= xRight;col++)
    {
      if (row>=0 && col>=0 && row<=127 && col<=127)
	{
	  if (steep)	
	    {
	      disp_buff[col][row] = clrLine;
	      for (i=col+1;i<=OLED_END;i++) disp_buff[i][row] = clrFill;
	    }
	  else 
	    {
	      disp_buff[row][col] = clrLine;
	      for (i=row+1;i<=OLED_END;i++) disp_buff[i][col] = clrFill;
	    }
	}
      error = error - dy;
      if (error<0)
	{
	  row = row + ystep;
	  error = error + dx;
	}
    }
}

void BarGraphValues(unsigned int _values[],unsigned char num_vals, unsigned char _clr)
{
  int i;
  char _num_string[15];
  for (i=1;i<=num_vals;i++)
    {
      if (_values[i]==_values[0])	DrawRectangle(GRAPH_LEFT+(i-1)*(unsigned int)GRAPH_RIGHT/num_vals,GRAPH_END-(_values[i]*(unsigned int)(GRAPH_END-GRAPH_START-12))/_values[0],GRAPH_LEFT+i*(unsigned int)GRAPH_RIGHT/num_vals,GRAPH_END,_clr<<1,_clr<<1);
      else DrawRectangle(GRAPH_LEFT+(i-1)*(unsigned int)GRAPH_RIGHT/num_vals,GRAPH_END-(_values[i]*(unsigned int)(GRAPH_END-GRAPH_START-12))/_values[0],GRAPH_LEFT+i*(unsigned int)GRAPH_RIGHT/num_vals,GRAPH_END,_clr,_clr);	//0x9CF3 or 7BEF
    }
  sprintf(_num_string,"max=%d",_values[0]);
  Draw_5x8_string(_num_string,strlen(_num_string),(GRAPH_RIGHT+GRAPH_LEFT)/2-strlen(_num_string)*3,GRAPH_START + 1,0xAF);
}

void LineGraphValues(unsigned int _values[],unsigned char num_vals, unsigned char _clr)
{
  int i;
  char _num_string[4];
	
  for (i=1;i<=num_vals;i++)
    {
      if (_values[i]==_values[0]) DrawLine(GRAPH_LEFT+((i-1)*(unsigned int)GRAPH_RIGHT/num_vals),GRAPH_END-(_values[i-1]*(i>1)*(unsigned int)(GRAPH_END-GRAPH_START-12))/_values[0],GRAPH_LEFT+((i)*(unsigned int)GRAPH_RIGHT/num_vals),GRAPH_END-(_values[i]*(unsigned int)(GRAPH_END-GRAPH_START-12))/_values[0],_clr);
      else DrawLine(GRAPH_LEFT+((i-1)*(unsigned int)GRAPH_RIGHT/num_vals),GRAPH_END-(_values[i-1]*(i>1)*(unsigned int)(GRAPH_END-GRAPH_START-12))/_values[0],GRAPH_LEFT+((i)*(unsigned int)GRAPH_RIGHT/num_vals),GRAPH_END-(_values[i]*(unsigned int)(GRAPH_END-GRAPH_START-12))/_values[0],_clr&0xDB);
    }
	
  sprintf(_num_string,"max=%d",_values[0]);
  Draw_5x8_string(_num_string,strlen(_num_string),(GRAPH_RIGHT+GRAPH_LEFT)/2-strlen(_num_string)*3,GRAPH_START + 1,0xAF);
}

void GraphAccel(int _values[],unsigned char num_vals, unsigned char _clr)
{
  int i;
  for (i=0;i<num_vals-1;i++)
    {
      DrawLine(((i*127)/num_vals),64-(_values[i]),(((i+1)*127)/num_vals),64-(_values[i+1]),_clr);
    }
}

//==============================================================================================
//=================================  Pong Functions  ===========================================
//==============================================================================================
void ClearPaddle(unsigned char _paddle,int _y_pos)
{
  int row;
  int col;
  if (_paddle == PLAYER) 
    {
      for (col= OLED_END - (PADDLE_X_OFFSET+PADDLE_THICKNESS); col<=(OLED_END - PADDLE_X_OFFSET);col++)
	{
	  for (row=_y_pos;row<=(_y_pos + PADDLE_HEIGHT);row++)
	    {
	      disp_buff[row][col] = 0x00;
	    }
	}
    }
  else 
    {
      for (col= PADDLE_X_OFFSET; col<=(PADDLE_X_OFFSET+PADDLE_THICKNESS);col++)
	{
	  for (row=_y_pos;row<=(_y_pos + PADDLE_HEIGHT);row++)
	    {
	      disp_buff[row][col] = 0x00;
	    }
	}
    }	
}

void DrawPaddle(unsigned char _paddle,int _y_pos)
{
  int row;
  int col;
  if (_paddle == PLAYER) 
    {
      for (col= OLED_END - (PADDLE_X_OFFSET+PADDLE_THICKNESS); col<=(OLED_END - PADDLE_X_OFFSET);col++)
	{
	  for (row=_y_pos;row<=(_y_pos + PADDLE_HEIGHT);row++)
	    {
	      disp_buff[row][col] = 0xFF;
	    }
	}
    }
  else 
    {
      for (col= PADDLE_X_OFFSET; col<=(PADDLE_X_OFFSET+PADDLE_THICKNESS);col++)
	{
	  for (row=_y_pos;row<=(_y_pos + PADDLE_HEIGHT);row++)
	    {
	      disp_buff[row][col] = 0xFF;
	    }
	}
    }
}

void ClearBall(int _x_pos,int _y_pos)
{
  int row;
  int col;
  for (col= _x_pos; col<=(_x_pos + BALL_THICKNESS);col++)
    {
      for (row=_y_pos;row<=(_y_pos + BALL_THICKNESS);row++)
	{
	  disp_buff[row][col] = 0x00;
	}
    }
}

void DrawBall(int _x_pos,int _y_pos)
{
  int row;
  int col;
  for (col= _x_pos; col<=(_x_pos + BALL_THICKNESS);col++)
    {
      for (row=_y_pos;row<=(_y_pos + BALL_THICKNESS);row++)
	{
	  disp_buff[row][col] = 0xA0;
	}
    }
}

void DrawScore(int comp_score,int player_score)
{
  char _score_string[15];
  sprintf(_score_string,"%d | %d",comp_score,player_score);
  Draw_8x12_string(_score_string,strlen(_score_string),(OLED_END)/2-(strlen(_score_string)*9)/2,PONG_SCORE_TOP,0x0F);
}


/****************************************************************
 *	CheckBallCollision:
 *					returns:	3 on end of game
 *								2 on top or bottom collision
 *								1 on paddle collision
 *								0 on no collision
 *
 ****************************************************************/
char CheckBallCollision(int _x_pos,int _y_pos,int _x_spd, int _y_spd)
{
  int i,j;
  if (((_x_pos+BALL_THICKNESS+_x_spd)>= (OLED_END))|| (_x_pos+_x_spd<= 0)) return 3;
  if ((_y_pos+BALL_THICKNESS+_y_spd)>=OLED_END || (_y_pos + _y_spd)<=PONG_TOP) return 2;
  if (_x_spd>0)
    {
      if (_y_spd>=0)
	{
	  for (i=0;i<=_x_spd;i++)
	    {
	      for (j=0;j<=_y_spd;j++)
		{
		  if (disp_buff[_y_pos+BALL_THICKNESS+j][_x_pos+BALL_THICKNESS+i]==0xFF) return 1;
		}
	    }
	}
      if (_y_spd<=0)
	{
	  for (i=0;i<=_x_spd;i++)
	    {
	      for (j=_y_spd+_y_pos;j<=_y_pos;j++)
		{
		  if (disp_buff[j][_x_pos+BALL_THICKNESS+i]==0xFF) return 1;
		}
	    }
	}
    }
  else
    {
      if (_y_spd>=0)
	{
	  for (i=_x_spd+_x_pos;i<=_x_pos;i++)
	    {
	      for (j=0;j<=_y_spd;j++)
		{
		  if (disp_buff[_y_pos+BALL_THICKNESS+j][i]==0xFF) return 1;
		}
	    }
	}
      if (_y_spd<=0)
	{
	  for (i=_x_spd+_x_pos;i<=_x_pos;i++)
	    {
	      for (j=_y_spd+_y_pos;j<=_y_pos;j++)
		{
		  if (disp_buff[j][i]==0xFF) return 1;
		}
	    }
	}
    }
  return 0;
}

void ClearDispBuff(void)
{
  int row;
  int col;
  for (row= 0; row<=OLED_END;row++)
    {
      for (col=0;col<=OLED_END;col++)
	{
	  disp_buff[row][col] = 0x00;
	}
    }
}


//==============================================================================================
//=============================== Logg Display Functions  ======================================
//==============================================================================================
void ShowLogging(void)
{
  ClearDispBuff();
  Draw_8x12_string("Logging",strlen("Logging"),(OLED_END)/2-(strlen("Logging")*9)/2,20,0xFF);
  Draw_8x12_string("Enter to Stop",strlen("Enter to Stop"),(OLED_END)/2-(strlen("Enter to Stop")*9)/2,40,0xFF);
}

//==============================================================================================
//================================  Debug Functions  ===========================================
//==============================================================================================

void RandomCircles(int num)
{
  int i;
  for(i = 0;i < num;i++)
    {		
      circle(rand()%130,rand()%130,rand()%64,rand(),rand());
      BufferToScreen(0,0,127,127);
      delay_ms(50);
    }
}


//==============================================================================================
//================================  Time Functions  ============================================
//==============================================================================================
void ShowTime(void)
{
  char time_string[30];
  struct accel_struct log_struct;
	
  int _read_val = 0;
	
	
  log_struct = accel_all();
  log_struct.accel_x = log_struct.accel_x*64/220;
  log_struct.accel_y = log_struct.accel_y*64/220;
  log_struct.accel_z = log_struct.accel_z*64/220;
	
  ClearDispBuff();
  sprintf(time_string,"%d/%d/%d",(int)MONTH,(int)DOM,(int)YEAR);
  Draw_8x12_string(time_string,strlen(time_string),(OLED_END)/2-(strlen(time_string)*9)/2 + log_struct.accel_x,53+log_struct.accel_y,0xFF);
	
  sprintf(time_string,"%d:%02d:%02d",(int)HOUR,(int)MIN,(int)SEC);
  Draw_8x12_string(time_string,strlen(time_string),(OLED_END)/2-(strlen(time_string)*9)/2 + log_struct.accel_x,73+log_struct.accel_y,0xFF);
	
  BufferToScreen(0,0,127,127);
	
  delay_ms(200);
	
  while (!_read_val)
    {
      log_struct = accel_all();
      log_struct.accel_x = log_struct.accel_x*64/220;
      log_struct.accel_y = log_struct.accel_y*64/220;
      log_struct.accel_z = log_struct.accel_z*64/220;
		
      ClearDispBuff();
      sprintf(time_string,"%d/%d/%d",(int)MONTH,(int)DOM,(int)YEAR);
      Draw_8x12_string(time_string,strlen(time_string),(OLED_END)/2-(strlen(time_string)*9)/2 + log_struct.accel_x,53+log_struct.accel_y,0xFF);

      sprintf(time_string,"%d:%02d:%02d",(int)HOUR,(int)MIN,(int)SEC);
      Draw_8x12_string(time_string,strlen(time_string),(OLED_END)/2-(strlen(time_string)*9)/2 + log_struct.accel_x,73+log_struct.accel_y,0xFF);
		
      BufferToScreen(0,0,127,127);
		
      delay_ms(5);
		
#ifdef USE_SLOW_GPIO
      _read_val = ~(IOPIN0>>SWITCH_ENTER);
#else
      _read_val = ~(FIO0PIN>>SWITCH_ENTER);
#endif
      _read_val &= 0x01;
    }
}

//==============================================================================================
//================================  I/O Functions  =============================================
//==============================================================================================


void write_c(unsigned char out_command)
{	
  //FIO1DIR |= LCD_DATA;
  FIO1PIN2 = out_command;
  FIO1CLR = LCD_DC;	//	|LCD_CS
  FIO1SET = LCD_RD;
  FIO1CLR = LCD_RD;
}

void write_d(unsigned char out_data)
{
  //FIO1DIR |= LCD_DATA;
  FIO1PIN2 = out_data;
  FIO1SET = LCD_DC;
  FIO1SET = LCD_RD;
  FIO1CLR = LCD_RD;
}

void write_color(unsigned short out_color)
{
  FIO1PIN2 = out_color>>8;
  FIO1SET = LCD_DC;
  FIO1SET = LCD_RD;
  FIO1CLR = LCD_RD;
  FIO1PIN2 = out_color&0xFF;
  FIO1SET = LCD_RD;
  FIO1CLR = LCD_RD;	
}

//unsigned char read_d(void)
//{	
//	FIO1DIR &= ~LCD_DATA;	
//	unsigned char _val;
//	FIO1SET = LCD_DC|LCD_RW;
//	//FIO0CLR |= LCD_CS;
//	FIO1SET = LCD_RD;
//	//_val = read_LCD_port();
//	_val = FIO1PIN2;
//	FIO1CLR = LCD_RD;
//	//FIO0SET |= LCD_CS;
//	return _val;
//}

void Reset_SSD1339(void)
{
  FIO1CLR = LCD_RSTB;
  delay_ms(100);
  FIO1SET = LCD_RSTB;
}
