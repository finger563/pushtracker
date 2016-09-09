#include "pong.h"
#include "main.h"
#include "OLED.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

int  _ball_x_spd;
int _ball_y_spd;
int computer_spd;
int player_spd;
int player_pos;
int computer_pos;
int ball_x_pos;
int ball_y_pos;
int player_score;
int computer_score;

void Pong(void)
{
	player_score=0;
	computer_score=0;
	_ball_x_spd =  2;
	_ball_y_spd =  2;
	InitPong();
	delay_ms(500);
	#ifdef USE_SLOW_GPIO
	int _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
	#else
	int _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
	#endif
	while (_read_state != 1)
	{
		//ClearDispBuff();
		UpdatePaddles();
		UpdateBall();
		//DrawScore(computer_score,player_score);
		BufferToScreen(0,0,OLED_END,OLED_END);
		#ifdef USE_SLOW_GPIO
		_read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
		#else
		_read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
		#endif
	}
}

void InitPong(void)
{
	if (_ball_y_spd<0) _ball_x_spd =  -_ball_x_spd;
	if (_ball_x_spd<0) _ball_y_spd =  -_ball_x_spd;
	computer_spd = 0;
	player_spd = 0;
	player_pos = PADDLE_START;
	computer_pos = PADDLE_START;
	ball_x_pos = BALL_X_START;
	ball_y_pos = BALL_Y_START;
	ClearDispBuff();
	DrawPaddle(COMPUTER,computer_pos);
	DrawPaddle(PLAYER,player_pos);
	DrawBall(ball_x_pos,ball_y_pos);
	DrawScore(computer_score,player_score);
	BufferToScreen(0,0,OLED_END,OLED_END);
}

void UpdateScore(unsigned char _paddle)
{
	if (_paddle == PLAYER) player_score++;
	else computer_score++;
}

void UpdateBall(void)
{
	char collision = CheckBallCollision(ball_x_pos,ball_y_pos,_ball_x_spd,_ball_y_spd);
	ClearBall(ball_x_pos,ball_y_pos);
	switch (collision)
	{
		case 0:
			ball_x_pos += _ball_x_spd;
			ball_y_pos += _ball_y_spd;
			DrawBall(ball_x_pos,ball_y_pos);
			break;
		case 1:
			_ball_x_spd = -(_ball_x_spd);
			if (ball_x_pos<60) _ball_y_spd = (_ball_y_spd+computer_spd)%BALL_MAX_SPD;
			else _ball_y_spd = (_ball_y_spd+player_spd)%BALL_MAX_SPD;
			ball_x_pos += _ball_x_spd;
			ball_y_pos += _ball_y_spd;
			DrawBall(ball_x_pos,ball_y_pos);			
			break;
		case 2:
			_ball_y_spd = -(_ball_y_spd);
			ball_x_pos += _ball_x_spd;
			ball_y_pos += _ball_y_spd;
			DrawBall(ball_x_pos,ball_y_pos);	
			break;
		case 3:
			if (_ball_x_spd<0) UpdateScore(PLAYER);
			else UpdateScore(COMPUTER);
			InitPong();
			delay_ms(1000);
			break;
		default:
			break;
	}
}

void UpdatePaddles(void)
{	
	ClearPaddle(COMPUTER,computer_pos);
	if (ball_y_pos<=(computer_pos+(PADDLE_HEIGHT)/2) && _ball_y_spd<0) computer_spd = -COMPUTER_MAX_SPD;
	else 
	{
		if (ball_y_pos>=(computer_pos+(PADDLE_HEIGHT)/2) && _ball_y_spd>0) computer_spd = COMPUTER_MAX_SPD;
		else 
		{
			computer_spd = 0;
		}
	}
	computer_pos = computer_pos+computer_spd;
	if ((computer_pos+PADDLE_HEIGHT)>(127)) computer_pos = (127-PADDLE_HEIGHT);
	if (computer_pos<PONG_TOP) computer_pos = PONG_TOP;
	DrawPaddle(COMPUTER,computer_pos);
	
	#ifdef USE_SLOW_GPIO
	int _read_val = ~(IOPIN0>>SWITCH_UP);
	#else
	int _read_val = ~(FIO0PIN>>SWITCH_UP);
	#endif
	_read_val &= 0x07;
	switch (_read_val)
	{
		case 0x04:
			ClearPaddle(PLAYER,player_pos);
			player_spd = PLAYER_MAX_SPD;
			player_pos = player_pos-player_spd;
			if ((player_pos+PADDLE_HEIGHT)>(127)) player_pos = (127-PADDLE_HEIGHT);
			if (player_pos<PONG_TOP) player_pos = PONG_TOP;
			DrawPaddle(PLAYER,player_pos);
			break;
		case 0x01:
			ClearPaddle(PLAYER,player_pos);
			player_spd = PLAYER_MAX_SPD;
			player_pos = player_pos+player_spd;
			if ((player_pos+PADDLE_HEIGHT)>(127)) player_pos = (127-PADDLE_HEIGHT);
			if (player_pos<PONG_TOP) player_pos = PONG_TOP;
			DrawPaddle(PLAYER,player_pos);
			break;
		case 0:
			player_spd = 0;
			break;
		default:
			break;
	}
}
