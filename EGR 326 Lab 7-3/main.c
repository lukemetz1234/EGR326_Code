/***********************************************************************************************************
  Title:        EGR 326 Lab 7
  Filename:     Part_3.c, wstep.h, wstep.c
  Author(s):    Mathew J. Yerian-French, Luke Metz
  Date:         10/16/2020
  Instructor:   Professor Brian Krug
  Description:  Part 3 of Lab 7. This code is the slave code that recieves data through I2C from another MSP432.
                The data is decoded and used to tell a stepper motor to move N steps in the correct direction transmitted
                from the master.
************************************************************************************************************
WIRING FOR MSP432
White Stepper : IN1 - P2.3 | IN2 - P5.1 | IN3 - P3.5 | IN4 - P3.7
I2C           : SDA - P1.6 | SCL - P1.7 | GND - GND
***********************************************************************************************************/

#include "msp.h"
#include "wstep.h"
#include <stdio.h>
#include <stdlib.h>

#define SLAVE_ADDRESS 0x48

char RXData;
int readData[3];
int count = 0;

void main(void){

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    initWhiteStepper(); //Stepper motor initialize
    SysTickInit();      //SysTick timer initialize

    P1->SEL0 |= BIT6 | BIT7; //P1.6 and P1.7 as UCB0SDA and UCB0SCL

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;      // Hold EUSCI_B0 module in reset state
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_SYNC;
    EUSCI_B0->I2COA0 = SLAVE_ADDRESS | EUSCI_B_I2COA0_OAEN;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // Clear SWRST to resume operation
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG0;       // Clear EUSCI_B0 RX interrupt flag
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE0;           // Enable EUSCI_B0 RX interrupt

    NVIC->ISER[0] = 0x00100000;                 // EUSCI_B0 interrupt is enabled in NVIC
    __enable_irq();     // All interrupts are enabled

    int value = 0;  //Value read from I2C
    int dir = 0;    //0 for positive, 1 for negative
    int position = 0;   //Total position of stepper motor

    while(1) {
        if(count == 3) {        //After 3 values have been recieved
            if(readData[0] == 45) { //If the first value is a "-"
                value = -4 * ((readData[1] - 48)*10 + (readData[2] - 48));  //Convert to decimal from ascii
                dir = 1; //Negative rotation
            }
            else {
                value = 4 * ((readData[1] - 48)*10 + (readData[2] - 48));   //Convert to decimal from ascii
                dir = 0; //Positive rotation
            }

            position += value;      //Keep track of the total position
            step(abs(value), dir);  //Move the stepper motor in the proper direction
            printf("Current position: %d\n", position);   //Print the current position
            count = 0;  //Reset for reading next I2C data transmission

        }
    }
}

void EUSCIB0_IRQHandler(void){
    uint32_t status = EUSCI_B0->IFG;        // Get EUSCI_B0 interrupt flag
    EUSCI_B0->IFG  &=~ EUSCI_B_IFG_RXIFG0;  // Clear EUSCI_B0 RX interrupt flag

    if(status & EUSCI_B_IFG_RXIFG0){        // Check if receive interrupt occurs
        RXData = EUSCI_B0->RXBUF;           // Load current RXData value to transmit buffer
        readData[count] = RXData;       //Store the read data into the readData array
    }
    count++;    //Increment to next position in array
}
