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

    P1->SEL0 |= BIT6 | BIT7;                // P1.6 and P1.7 as UCB0SDA and UCB0SCL

    initWhiteStepper();
    SysTickInit();

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;      // Hold EUSCI_B0 module in reset state
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_SYNC;
    EUSCI_B0->I2COA0 = SLAVE_ADDRESS | EUSCI_B_I2COA0_OAEN;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;        // Clear SWRST to resume operation
    EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG0;       // Clear EUSCI_B0 RX interrupt flag
    EUSCI_B0->IE |= EUSCI_B_IE_RXIE0;            // Enable EUSCI_B0 RX interrupt

    NVIC->ISER[0] = 0x00100000;                 // EUSCI_B0 interrupt is enabled in NVIC
    __enable_irq();                  // All interrupts are enabled

    int value = 0;
    int dir = 0; //0 for positive, 1 for negative
    int position = 0;

    while(1) {
        if(count == 3) {
            if(readData[0] == 45) {
                value = -4 * ((readData[1] - 48)*10 + (readData[2] - 48));
                dir = 1; //Negative rotation
            }
            else {
                value = 4 * ((readData[1] - 48)*10 + (readData[2] - 48));
                dir = 0; //Positive rotation
            }

            position += value;
            step(abs(value), dir);
            printf("%d\n", position);
            count = 0;

        }
    }
}

void EUSCIB0_IRQHandler(void){
    uint32_t status = EUSCI_B0->IFG;         // Get EUSCI_B0 interrupt flag
    EUSCI_B0->IFG  &=~ EUSCI_B_IFG_RXIFG0;      // Clear EUSCI_B0 RX interrupt flag

    if(status & EUSCI_B_IFG_RXIFG0){             // Check if receive interrupt occurs
        RXData = EUSCI_B0->RXBUF;         // Load current RXData value to transmit buffer

        //printf("%d\n", RXData);
        readData[count] = RXData;
    }
    count++;
}
