#include "msp.h"
#include <wstep.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

void initWhiteStepper(void){
    IN1PORT->SEL0 &= ~(IN1PIN);    //setting up In1
    IN1PORT->SEL1 &= ~(IN1PIN);
    IN1PORT->DIR |= (IN1PIN);
    IN1PORT->OUT |= (IN1PIN);

    IN2PORT->SEL0 &= ~(IN2PIN);    //setting up In2
    IN2PORT->SEL1 &= ~(IN2PIN);
    IN2PORT->DIR |= (IN2PIN);
    IN2PORT->OUT |= (IN2PIN);

    IN3PORT->SEL0 &= ~(IN3PIN);    //setting up In3
    IN3PORT->SEL1 &= ~(IN3PIN);
    IN3PORT->DIR |= (IN3PIN);
    IN3PORT->OUT |= (IN3PIN);

    IN4PORT->SEL0 &= ~(IN4PIN);    //setting up In4
    IN4PORT->SEL1 &= ~(IN4PIN);
    IN4PORT->DIR |= (IN4PIN);
    IN4PORT->OUT |= (IN4PIN);
}


void step(int steps, int direction){//input is direction and number of steps
    int count = 0;
    //Comment out for half step mode
    steps = steps * 2;//due to the half step method
volatile enum stepper{//count and corner sequence
    Step1, Step2,
    Step3, Step4
} step = Step1;
//this is the half step method
while(1){
    if(steps <= 80000){
    switch(step)
     {
         case Step1:
            if(count >= steps) break;
            IN1PORT->OUT |= (IN1PIN);
            IN2PORT->OUT |= (IN2PIN);
            IN3PORT->OUT |= (IN3PIN);
            IN4PORT->OUT &= ~(IN4PIN);
            delay_micro(2000);
            count++;
            if(direction == 0){ step = Step2; break;}
            if(direction == 1){ step = Step4; break;}
         case Step2:
            if(count >= steps){break;}
            IN1PORT->OUT &= ~(IN1PIN);
            IN2PORT->OUT |= (IN2PIN);
            IN3PORT->OUT |= (IN3PIN);
            IN4PORT->OUT |= (IN4PIN);
            delay_micro(2000);
            count++;
            if(direction == 0){ step = Step3; break;}
            if(direction == 1){ step = Step1; break;}
         case Step3:
             if(count >= steps){break;}
             IN1PORT->OUT |= (IN1PIN);
             IN2PORT->OUT &= ~(IN2PIN);
             IN3PORT->OUT |= (IN3PIN);
             IN4PORT->OUT |= (IN4PIN);
             delay_micro(2000);
             count++;
             if(direction == 0){ step = Step4; break;}
             if(direction == 1){ step = Step2; break;}
         case Step4:
             if(count >= steps){break;}
             IN1PORT->OUT |= (IN1PIN);
             IN2PORT->OUT |= (IN2PIN);
             IN3PORT->OUT &= ~(IN3PIN);
             IN4PORT->OUT |= (IN4PIN);
             delay_micro(2000);
             count++;
             if(direction == 0){ step = Step1; break;}
             if(direction == 1){ step = Step3; break;}
         default:
             if(count >= steps) break;
             step = Step1;
     }
    }
    else break;
    if(count >= steps) break;
}
}

void SysTickInit() {
    SysTick->CTRL = 0; //Disable systick while setting it up
    SysTick->LOAD = 0x00FFFFFF; //Sets maximum reload value
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000005; //No interrupts, Systick enabled
}

void delay_ms(int ms) {
    SysTick->LOAD = ms*3000;       //millisecond timer
    SysTick->VAL = 0;                      //Starts the timer
    while ((SysTick->CTRL & 0x00010000) == 0);
}

void delay_micro(int  mu) {
    SysTick->LOAD = mu*3;       //microsecond timer
    SysTick->VAL = 0;           //Starts the timer counting
    while ((SysTick->CTRL & 0x00010000) == 0); //
}
