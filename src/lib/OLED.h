/******************************************************************************
 *
 * $RCSfile$
 * $Revision: 124 $
 *
 * This module provides the interface definitions for setting up and
 * controlling the various interrupt modes present on the ARM processor.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#ifndef OLED_H
#define OLED_H

//#define SSD1339
#define SSD1351		// for newer screens with 1351 gdd



#ifdef SSD1339
// data bus for LCD, pins on port 0
#define D0 16
#define D1 17
#define D2 18
#define D3 19
#define D4 20
#define D5 21
#define D6 22
#define D7 23
#define D8 24

// OLED data port
#define LCD_DATA ((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7)|(1<<D8))

// other OLED pins
#define LCD_RD    (1<<25)			// P1.25
#define LCD_RW    (1<<26)			// P1.26
#define LCD_CS    (1<<27)			// P1.27
#define LCD_DC    (1<<28)			// P1.28
#define LCD_RSTB  (1<<29)			// P1.29

#define BS1			(1<<30)			// P1.30
#define BS2			(1<<31)			// P1.31

//==========================================
//===========If Using new OLED==============
//==========================================
#else
// data bus for LCD, pins on port 0
#define D0 16
#define D1 17
#define D2 18
#define D3 19
#define D4 20
#define D5 21
#define D6 22
#define D7 23

// OLED data port
#define LCD_DATA ((1<<D0)|(1<<D1)|(1<<D2)|(1<<D3)|(1<<D4)|(1<<D5)|(1<<D6)|(1<<D7))

// other OLED pins
#define LCD_RD    (1<<25)			// P1.25
#define LCD_RW    (1<<26)			// P1.26
#define LCD_CS    (1<<27)			// P1.27
#define LCD_DC    (1<<28)			// P1.28
#define LCD_RSTB  (1<<29)			// P1.29

#define BS0			(1<<30)			// P1.30
#define BS1			(1<<31)			// P1.31
#endif

#define OLED_END 	(unsigned int)127
#define GRAPH_START	(unsigned int)45
#define GRAPH_END	(unsigned int)115
#define GRAPH_RIGHT	(unsigned int)127
#define GRAPH_LEFT	(unsigned int)0
#define TIME_HEIGHT	(unsigned int)120
#define TEXT_OFFSET	(unsigned int)5
#define TEXT_SPACING	(unsigned int)4
#define VAL_OFFSET	(unsigned int)90
#define TEXT_HEIGHT	(unsigned int)45


#define MAX_VARIABLES		10
#define NUM_MEASUREMENTS	4

#define RST_INDEX			4
#define PONG_INDEX			5
#define GRAPH_INDEX			6
#define LOG_INDEX			7
#define CLOCK_INDEX			8
#define PWR_INDEX			9


#define DAY_SIZE		(unsigned char)24
#define WEEK_SIZE		(unsigned char)42
#define MONTH_SIZE		(unsigned char)32
#define YEAR_SIZE		(unsigned char)52


#define DOWN		0
#define UP			1

#define PADDLE_THICKNESS	(int)0x02
#define PADDLE_HEIGHT		(int)0x18
#define PADDLE_X_OFFSET		(int)0x02
#define PADDLE_START		((int)OLED_END-PADDLE_THICKNESS)/2
#define BALL_X_START		(int)60
#define BALL_Y_START		(int)60
#define BALL_THICKNESS		(int)5

#define PONG_TOP			(int)15
#define PONG_SCORE_TOP		(int)(PONG_TOP - 12)

#define COMPUTER	0
#define PLAYER		1

#define PLAYER_MAX_SPD		(int)0x07
#define COMPUTER_MAX_SPD	(int)0x05
#define BALL_MAX_SPD		(int)0x08


// inialize OLED
void OLED_init(void);
void OLED_ShutDown(void);
void OLED_TurnOn(void);
void ClearScreen(void);
void ClearWindow(unsigned char x1,unsigned char y1, unsigned char x2, unsigned char y2);

// reset Controller
void Reset_SSD1339(void);

// write command or data
void write_c(unsigned char out_command);
void write_d(unsigned char out_data);
void write_color(unsigned short out_color);
//unsigned char read_d(void);

void Draw_8x12_char(unsigned char* _char_matrix,int x_start,int y_start,unsigned char clr);
void Write_8x12_char(unsigned char* _char_matrix,int x_start,int y_start,unsigned char clr);
void Draw_8x12_string(unsigned char* str,unsigned char len,int x_start,int y_start,unsigned char clr);
void Write_8x12_string(unsigned char* str,unsigned char len,int x_start,int y_start,unsigned char clr);
void Draw_5x8_char(unsigned char* _char_matrix,int x_start,int y_start,unsigned char clr);
void Draw_5x8_string(unsigned char* str,unsigned char len,int x_start,int y_start,unsigned char clr);

void DrawRectangle(unsigned char xLeft, unsigned char yTop, unsigned char xRight, unsigned char yBottom, unsigned char clrOutline, unsigned char clrFill);
//void ScreenToBuffer(unsigned char x_start,unsigned char y_start,unsigned char x_end, unsigned char y_end);
void BufferToScreen(unsigned char x_start,unsigned char y_start,unsigned char x_end, unsigned char y_end);
void ClearGraph(void);
void BarGraphValues(unsigned int _values[],unsigned char num_vals, unsigned char _clr);
void LineGraphValues(unsigned int _values[],unsigned char num_vals, unsigned char _clr);
void DrawLine(int xLeft, int yTop, int xRight, int yBottom, unsigned char clrFill);
void DrawLineFill(int xLeft, int yTop, int xRight, int yBottom, unsigned char clrLine, unsigned char clrFill);
void GraphAccel(int _values[],unsigned char num_vals, unsigned char _clr);

void RandomCircles(int num);
void circle(int cx, int cy, int radius, unsigned char clroutline,unsigned char clrfill); 
void plot8points(int cx, int cy, int x, int y, unsigned char clroutline,unsigned char clrfill);
void plot4points(int cx, int cy, int x, int y, unsigned char clroutline,unsigned char clrfill);
//=========  Menu Functions  ===========

unsigned char ScrollVariables(unsigned char _dir,unsigned char _spd);
unsigned char ScrollTimes(unsigned char _dir);

void DrawTimes(unsigned char clr);
void UpdateText(unsigned char text_row,unsigned char text_index,unsigned char clr);
void ClearText(void);
void DrawVariables(void);
void ClearTimes(void);


//=========  Pong Functions  ===========
void DrawPaddle(unsigned char _paddle,int _y_pos);
void DrawBall(int _x_pos,int _y_pos);
void ClearPaddle(unsigned char _paddle,int _y_pos);
void ClearBall(int _x_pos,int _y_pos);
void DrawScore(int comp_score,int player_score);
void ClearDispBuff(void);
char CheckBallCollision(int _x_pos,int _y_pos,int _x_spd,int _y_spd);

//========== Logging Functions ==========
void ShowLogging(void);


//========== Time Functions ==========
void ShowTime(void);


#endif
