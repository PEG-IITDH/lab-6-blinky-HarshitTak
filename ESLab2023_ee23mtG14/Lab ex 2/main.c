#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#define SYST_CSR             (*((volatile uint32_t *)0xE000E010))
#define SYST_RVR             (*((volatile uint32_t *)0xE000E014))
#define SYST_CVR             (*((volatile uint32_t *)0xE000E018))

/**
 * main.c
 */

void delay(int num_sec)
{
    //clock_t start_time = clock();

    //while(clock() <= start_time + num_msec);
    int i;
    int j;

    for(i=0;i<num_sec;i++)
    {
        for(j=0; j<num_sec; j++);
    }
}

  void delay1(int ms)
       {
           SYST_CSR&=0x00;
           SYST_RVR=ms;
           SYST_CSR|=0x01;
           while((SYST_CSR==0x00001))
           {

           }
           SYST_CSR&=0x00;
       }



int main(void)
{
    SYSCTL_RCGC2_R |= 0x00000020;;      /* enable clock to GPIOF */
        GPIO_PORTF_LOCK_R = 0x4C4F434B;     /* unlock commit register */
        GPIO_PORTF_CR_R = 0x01;             /* make PORTF0 configurable */
        GPIO_PORTF_DIR_R = 0x0E;            /* set PORTF3+PORTF2+PORTF1 pin as output (LED) pin */
                                            /* and PORTF4 and PORTF0 as input, SW1 is on PORTF4  and SW2 is PORTF0*/
        GPIO_PORTF_DEN_R = 0x1F;            /* set PORTF pins 4-3-2-1 as digital pins */
        GPIO_PORTF_PUR_R = 0x11;            /* enable pull up for pin 4 and 0 */



        int led_count = 0; //initial state of count
        int x;
        int led_array[3] = {0x02, 0x04, 0x08}; //possible states of LED
        int old = 0;
        int new = 0;

        while(1)
        {
            x = GPIO_PORTF_DATA_R; //reading the value of PORTF to a variable
            delay1(4000);
            GPIO_PORTF_DATA_R = led_array[led_count]; //Switching on the LEDs for the first time

            if(!(x & 0x01))
            {
                new = 1;
            }
            else
            {
                new = 0;
            }

            if(old == 0 && new == 1)
            {
                GPIO_PORTF_DATA_R = led_array[led_count]; //turn on all LEDs
                if(led_count < 2)
                {
                    led_count = led_count + 1;
                }
                else
                {
                    led_count = 0;
                }
            }
            old = new;


            //delay(1000); //delay for differentiating lights
            /*
            if(led_count < 2)
            {
                led_count = led_count + 1;
            }
            else
            {
                led_count = 0;
            }
            */
        }
}
