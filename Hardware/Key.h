#ifndef __KEY_H
#define __KEY_H

void Key_Init(void);
uint8_t Key_GetNum(void);
uint8_t Key_GetMode(void);	//(0=pid速度跟随;1=角度跟随)

#endif
