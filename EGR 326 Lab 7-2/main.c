/***********************************************************************************************************
  Title:        EGR 326 Lab 7
  Filename:     Part_2.c
  Author(s):    Mathew J. Yerian-French, Luke Metz
  Date:         10/16/2020
  Instructor:   Professor Brian Krug
  Description:  Part 2 of Lab 7. This code is the slave code that receives data through I2C from another MSP432.
                The data is used to change the color of the on board LED of the slave MSP432. The master sends either
                an R, G, or B character and this is decoded and used to set the correct LED on.
************************************************************************************************************
WIRING FOR MSP432
I2C           : SDA - P1.6 | SCL - P1.7 | GND - GND
RGB LEDs       : RED - P2.0 | GREEN - P2.1 | BLUE P2.2
***********************************************************************************************************/

#include "msp.h"

#define LED2_RED BIT0
#define LED2_GREEN BIT1
#define LED2_BLUE BIT2

#define SLAVE_ADDRESS 0x48

char RXData;

void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    P2->DIR = LED2_RED|LED2_GREEN|LED2_BLUE;    //LED init
    P2->OUT = 0x00; //All LEDs off
    P1->SEL0 |= BIT6 | BIT7;    // P1.6 and P1.7 as UCB0SDA and UCB0SCL

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;      // Hold EUSCI_B0 module in reset state
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_SYNC;
    EUSCI_B0->I2COA0 = SLAVE_ADDRESS | EUSCI_B_I2COA0_OAEN;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // Clear SWRST to resume operation
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG0;       // Clear EUSCI_B0 RX interrupt flag
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE0;           // Enable EUSCI_B0 RX interrupt

    NVIC->ISER[0] = 0x00100000;     // EUSCI_B0 interrupt is enabled in NVIC
    __enable_irq();                 // All interrupts are enabled

    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;         // Sleep on exit
    __sleep();                  // enter LPM0
}

void EUSCIB0_IRQHandler(void){
    uint32_t status         = EUSCI_B0->IFG;         // Get EUSCI_B0 interrupt flag
    EUSCI_B0->IFG  &=~ EUSCI_B_IFG_RXIFG0;      // Clear EUSCI_B0 RX interrupt flag

    if(status & EUSCI_B_IFG_RXIFG0){    // Check if receive interrupt occurs
           RXData = EUSCI_B0->RXBUF;    // Load current RXData value to transmit buffer

        if (RXData == 'R'){
            P2->OUT |= LED2_RED;    // Red LED on
            P2->OUT &= ~LED2_GREEN;
            P2->OUT &= ~LED2_BLUE;
        }
        else if(RXData == 'G'){
            P2->OUT |= LED2_GREEN;  // Green LED on
            P2->OUT &= ~LED2_RED;
            P2->OUT &= ~LED2_BLUE;
        }
        else if(RXData == 'B'){
            P2->OUT |= LED2_BLUE;   // Blue LED on
            P2->OUT &= ~LED2_GREEN;
            P2->OUT &= ~LED2_RED;
        }
    }
}
