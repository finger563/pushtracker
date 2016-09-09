#include "menu.h"
#include "OLED.h"
#include "main.h"
#include "pong.h"

unsigned char menu_time;
unsigned char menu_var;
unsigned char _graph_lengths[4] = {DAY_SIZE,WEEK_SIZE,MONTH_SIZE,YEAR_SIZE};
extern unsigned int _graph[55];

void InitMenu(void)
{
	DrawVariables();
	UpdateGraph();
}

void MenuAction(unsigned char _source)
{
	switch (_source)
	{
		case SWITCH_UP:
			menu_var = ScrollVariables(0,3);
			break;
		case SWITCH_DOWN:
			menu_var = ScrollVariables(1,3);
			break;
		case SWITCH_ENTER:
			if (menu_var<NUM_MEASUREMENTS) menu_time = ScrollTimes(0);
			if (menu_var==RST_INDEX) reset_system();
			if (menu_var==PONG_INDEX) 
			{
				Pong();
				InitMenu();
			}
			if (menu_var==GRAPH_INDEX)
			{
				GraphAccelVals();
				InitMenu();
			}
			if (menu_var==LOG_INDEX)
			{
				ShowLogging();
				BufferToScreen(0,0,127,127);
				LogData();
				InitMenu();
			}
			if (menu_var==CLOCK_INDEX)
			{
				ShowTime();
				InitMenu();
				delay_ms(50);
			}
			if (menu_var==PWR_INDEX)
			{
				OLED_ShutDown();
				#ifdef USE_SLOW_GPIO
				int _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
				while (_read_state != 1) _read_state = (~(IOPIN0>>SWITCH_ENTER))&0x01;
				#else
				int _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
				while (_read_state != 1) _read_state = (~(FIO0PIN>>SWITCH_ENTER))&0x01;
				#endif
				OLED_TurnOn();
				InitMenu();
			}
			break;
		default:
			break;
	}
	UpdateGraph();
}

void GraphAction(void)
{
	DrawTimes(0xAF);
	
	if (menu_var<NUM_MEASUREMENTS)
	{
		load_data(menu_time,menu_var);
		BarGraphValues(_graph,_graph_lengths[menu_time], GRAPH_COLOR);
	}
	BufferToScreen(0,GRAPH_START,OLED_END,OLED_END);
}

void UpdateGraph(void)
{
	ClearWindow(0,GRAPH_START, OLED_END, OLED_END);
	GraphAction();
}
