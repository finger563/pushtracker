#ifndef MENU_H
#define MENU_H

//#define GRAPH_COLOR		(unsigned short)0x00BB
#define GRAPH_COLOR		(unsigned char)0x27

void InitMenu(void);
void MenuAction(unsigned char _source);
void GraphAction(void);
void UpdateGraph(void);

#endif
