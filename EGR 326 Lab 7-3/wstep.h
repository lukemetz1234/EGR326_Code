#ifndef __WSTEP_H__
#define __WSTEP_H__

#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define IN1PORT P2
#define IN1PIN BIT3
#define IN2PORT P5
#define IN2PIN BIT1
#define IN3PORT P3
#define IN3PIN BIT5
#define IN4PORT P3
#define IN4PIN BIT7

void initWhiteStepper(void);
void step(int steps, int direction);


void SysTickInit(void);
void delay_ms(int ms);//delay in milliseconds using systick
void delay_micro(int mu);

#endif
