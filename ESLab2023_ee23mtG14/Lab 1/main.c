#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

/**
 * main.c
 */
int main(void)
{



    SYSCTL_RCGC2_R |= 0x00000020;;      /* enable clock to GPIOF */
        GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
        GPIO_PORTF_CR_R = 0x01;             /* make PORTF0 configurable */
        GPIO_PORTF_DIR_R = 0x0E;            /* set PORTF3+PORTF2+PORTF1 pin as output (LED) pin */
                                            /* and PORTF4 and PORTF0 as input, SW1 is on PORTF4  and SW2 is PORTF0*/
        GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4-3-2-1 as digital pins */
        GPIO_PORTF_PUR_R = 0x11;            /* enable pull up for pin 4 and 0 */

        GPIO_PORTF_DATA_R = 0x0;
        GPIO_PORTF_DATA_R = 0x2;
        GPIO_PORTF_DATA_R = 0x4;
        GPIO_PORTF_DATA_R = 0x8;
        GPIO_PORTF_DATA_R = 0xE;
        int i = 0,j = 0,k;
        while(1)
        {
            GPIOPinWrite(GPIO_PORTF_DATA_R = 0x2);
            for (i=0;i<100000;i++){k=1}
            GPIOPinWrite(GPIO_PORTF_DATA_R = 0x0);
            for (j=0;j<100000;j++){k=0};


        }




}
