# EE 690 Lab Report 8
### Group : 14
### Name : Harshit Tak and Miriyala Pranay Kamal
### ROLL NO. : EE23MT007 AND 200030033
### FACULTY ; Dr. ABHIJIT KSHIRSAGAR

## OBJECTIVE : 
 
Part 1:

Program a microcontroller to transmit:

"F0" if SW1 is pressed

"AA" if SW2 is pressed 

over UART with baud rate 9600 and odd parity. 

Your program should also listen for incoming data on the UART with the same baud and parity config; if "AA" is received LED should be GREEN; if "F0" is received, the LED should be BLUE and if any error is detected LED should be RED. Test this by communicating with your neighboring group.

### What is UART ?
UART is a common peripheral that allows the microcontroller to transmit and receive data in a serial format.

UART is a simple serial communication protocol that allows the transfer of data between 2 devices.

## Functional Description 
The UART is configured to transmit and receive via TEX and REX bits, which are set using UARTCTL(UART control) register.

### Initialization of UART :
1. Enabling the UART mode.  
2. Enable the clock to the appropriate GPIO module.
3. Setting the Alternate Function register.
4. Set the appropriate Baud Rate.
5. At Last setting the appropriate Register for performing the transmission and receiving of the data.

## Code : 

#include <stdint.h>
#include "tm4c123gh6pm.h"

//Mask bits for GPIO interrupts
#define MASK_BITS 0x11 // pins for which interrupts should be enabled

/*
 *  U2Rx 53 PD6 (1) I TTL UART module 2 received.
    U2Tx 10 PD7 (1) O TTL UART module 2 transmit
 */

//define LEDs
#define RED_LED 1
#define BLUE_LED 2
#define GREEN_LED 3

uint8_t Rx;

void UARTinit()
{
    SYSCTL_RCGCGPIO_R |= (1<<3) | (1<<5); //Enable and provide a clock to GPIO Port D in Run mode

    //Setting output pins - alt function - PORT D
    GPIO_PORTD_LOCK_R = 0x4C4F434B;         /* unlock commit register */
    GPIO_PORTD_CR_R = 0xFF;                 /* make PORTD0 configurable */
    GPIO_PORTD_AFSEL_R |= (1<<6) | (1<<7);  //Enable alternate functions for PD6 and PD7
    GPIO_PORTD_PCTL_R |= 0x11000000;        //Port Mux Control 7, Port Mux Control 6

    GPIO_PORTD_DIR_R &= ~0x40;              // Set Pin 6 as an input
    GPIO_PORTD_DIR_R |= 0x80;               // Set Pin 7 as an input
    GPIO_PORTD_PUR_R |= 0x40;               // Enable the internal pull-up resistor for Pin 0, 4
    GPIO_PORTD_DEN_R |= (0x80) | (0X40);    // Digital enable Pin 0, 4

    //UART initialization
    UART2_CTL_R &= (0<<0);                  //Disable UART
    UART2_IBRD_R = 0x68;                    //Integer Baud-Rate Divisor - 104
    UART2_FBRD_R = 0x0B;                    //Fractional Baud-Rate Divisor - 11
    UART2_LCRH_R |= (1<<6) | (1<<5) | (1<<1); //0x03 for 8 bit data mode, 2nd bit for parity enable
    UART2_LCRH_R &= (~(1<<2));              //Odd parity
    UART2_LCRH_R &= (~(1<<3));              //One stop bit
    UART2_CC_R = 0x0;                       //System clock
    UART2_CTL_R |= (1<<8);         //Transmit enable, UART enable
    UART2_CTL_R |= (1<<9);                  //Recieve enable
    UART2_CTL_R |= (1<<0);                  //UART enable
}

void GPIOinit()
{
    /* Setting of PF2 pin for MIPWM6 channel output pin */
    SYSCTL_RCGCGPIO_R |= (1<<5);            /* Enable system clock to PORTF */
    SYSCTL_RCGCUART_R |= (1<<2);            //Enable and provide a clock to UART module 2 in Run mode
    GPIO_PORTF_LOCK_R = 0x4C4F434B;         /* unlock commit register */
    GPIO_PORTF_CR_R = 0xFF;                 /* make PORTF0 configurable */
    GPIO_PORTF_DIR_R |= 0x0E;               // Set Pin 1, 2, 3 as an output
    GPIO_PORTF_DEN_R |= (1<<2) | (1<<1) | (1<<3);    /* set PF 1,2,3 as a digital pin */

    // Initialize GPIO for Button (assuming it's connected to PortF, Pin 4)
    GPIO_PORTF_DIR_R &= ~0x11;              // Set Pin 0, 4 as an input
    GPIO_PORTF_PUR_R |= 0x11;               // Enable the internal pull-up resistor for Pin 0, 4
    GPIO_PORTF_DEN_R |= 0x11;               // Digital enable Pin 0, 4

    //GPIO interrupts enable
    // Enable interrupt for the button (PortF, Pin 4)
    GPIO_PORTF_IM_R &= ~MASK_BITS;          // mask interrupt by clearing bits
    GPIO_PORTF_IS_R &= ~MASK_BITS;          // edge sensitive interrupts
    GPIO_PORTF_IBE_R &= ~MASK_BITS;         // interrupt NOT on both edges
    GPIO_PORTF_IEV_R &= ~MASK_BITS;         // falling edge trigger
    /* Prioritize and enable interrupts in NVIC */
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xF1FFFFFF) | (3 << 21);
     // interrupt priority register 7
     // bits 21-23 for interrupt 30 (port F)
    NVIC_EN0_R |= 1 << 30;                  // enable interrupt 30 (port F)
    /* Enable interrupt generation in GPIO */
    GPIO_PORTF_ICR_R = MASK_BITS;           // clear any prior interrupt
    GPIO_PORTF_IM_R |= MASK_BITS;           // unmask interrupt by setting bits
    //GPIO_PORTF_DATA_R = 0x02;             //Light LED
}

void GPIOF_Handler()                        //Deals with transmitting data
{
    GPIO_PORTF_ICR_R = 0x11;                // Clear the interrupt flag

    if (GPIO_PORTF_DATA_R & 0x01)
    {
        //UART2_CTL_R |= (1<<8) | (1<<0);   //Transmit enable, UART enable
        UART2_DR_R = 0xAA;                  //Switch 2
        //UART2_CTL_R &= (0<<8) | (0<<0);   //Transmit disable, UART disable
    }

    else if (GPIO_PORTF_DATA_R & 0x10)
    {
        //UART2_CTL_R |= (1<<8) | (1<<0);   //Transmit enable, UART enable
        UART2_DR_R = 0xF0;                  //Switch 1
        //UART2_CTL_R &= (0<<8) | (0<<0);   //Transmit disable, UART disable
    }
}

void UART_Receive()                         //Deals with recieving data
{
    if((UART2_RIS_R & (1<<4))==(1<<4))      //check if receive is done
    {
        //GPIO_PORTF_DATA_R |= (1<<BLUE_LED);     //Turn ON Blue LED - debug
        UART2_CTL_R &= ~(1<<0);             //UART Disable
        Rx = UART2_DR_R;
        // Check if the Data recieved is correct by transmitting it again on PD7 and probing

        UART2_DR_R = Rx;                    //Data Register
        UART2_CTL_R |= (1<<8);              //Tx Enable
        UART2_CTL_R |= (1<<0);              //UART Enable

        if(Rx == 0xAA)    //Case 1
        {
            GPIO_PORTF_DATA_R |= (1<<BLUE_LED);     //Turn ON Blue LED
            GPIO_PORTF_DATA_R &= ~(1<<GREEN_LED);   //Turn OFF Green LED
            GPIO_PORTF_DATA_R &= ~(1<<RED_LED);     //Turn OFF RED LED
        }

        else if(Rx == 0xF0)   //Case 2
        {
            GPIO_PORTF_DATA_R |= (1<<GREEN_LED);    //Turn ON Green LED
            GPIO_PORTF_DATA_R &= ~(1<<BLUE_LED);    //Turn OFF BLUE LED
            GPIO_PORTF_DATA_R &= ~(1<<RED_LED);     //Turn OFF RED LED
        }

        UART2_ICR_R |= (1<<10)|(1<<9)|(1<<8)|(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<1);    //Clear All Interrupts
        UART2_CTL_R &= ~(1<<0);                                                     //UART Disable
        UART2_LCRH_R &= ~(1<<4);                                                    //Flush the FIFO
        UART2_CTL_R |= (1<<0);                                                      //UART Enable
    }
}

void UART_Handler()
{
    if((UART2_RIS_R & (1<<10)) == (1<<10))      //Overrun Error Raw Interrupt Status
      {
          GPIO_PORTF_DATA_R |= (1<<RED_LED);    //Turn ON RED LED
      }
    if((UART2_RIS_R & (1<<9)) == (1<<9))        //Break Error Raw Interrupt Status
      {
          GPIO_PORTF_DATA_R |= (1<<RED_LED);    //Turn ON RED LED
      }
    if((UART2_RIS_R & (1<<8)) == (1<<8))        //Parity Error Raw Interrupt Status
      {
          GPIO_PORTF_DATA_R |= (1<<RED_LED);    //Turn ON RED LED
      }
    if((UART2_RIS_R & (1<<7)) == (1<<7))        //Framing Error Raw Interrupt Status
      {
          GPIO_PORTF_DATA_R |= (1<<RED_LED);    //Turn ON RED LED
      }
    if((UART2_RIS_R & (1<<6)) == (1<<6))        //Receive Time-Out Raw Interrupt Status
      {
          GPIO_PORTF_DATA_R |= (1<<RED_LED);    //Turn ON RED LED
      }

    UART2_ICR_R |= (1<<10)|(1<<9)|(1<<8)|(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<1);    //Clear Interrupts
    UART2_CTL_R &= ~(1<<0);                                                     //UART Disable
}


int main(void)
 {
    GPIOinit();     //Initialize GPIO port F(switches), D (UART Tx AND Rx)
    UARTinit();     //Initialize UART 2 and set Baud rate to 9600

    while(1)
    {
        UART_Receive();     //Check for UART data on UART2 Tx pin
    }
}


## Calculation :

System Clock Frequency (f) = 16 MHz
Baud Rate = 9600

BRD = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)

BRD = (16 * 10^6)/16*9600

BRD = BRDI + BRDF = 104.16667

BRDI = 104 and BRDF = 0.16667

for Calculating the value for UARTFBRRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
UARTFBRRD[DIVFRAC] = integer(0.16667 * 64 + 0.5) = 11


## Observation 

In the above experiment we perform the transmission and reception of the data in between 2 different boards using the TXE and RXE provided by the UART in the cortex m4 board. 
We program the board in accordance to the baud rate specified in the objective i.e, 9600. 
And using the baud rate we calculate the respective values and give the integral part to UARTIBRD and fraction part to UARTFBRD that needs to be initialized for the code functionality.
In the experiment we program the 2 board in such a way that the first board is responsible for the transmission of the specified data and the second board will receive the respective data and  Acknowledgment according to the received data by blinking the LED(Blue and Green) and also blink the Red LED whenever there is some error.
Whenever we press the switch 1 on the board1, 0xF0 is transmitted to the board and if the board2 receives the data without any error then the green LED will glow and if the switch2 is pressed then the blue LED will glow. In case of any error the red LED will glow. And to check that the transmitted data is wrong or write we will use the PD7 to transmit the data from board2 to the Oscilloscope and then observe the output as shown in the image below.

## Output :
### Output for F0 Transmite<br>

![Output for F0 Transmitee](https://github.com/PEG-IITDH/lab-6-blinky-HarshitTak/blob/main/Desktop/Lab_8_UART/IMAGES/F0_transmit.png)

### Output for AA Transmite<br>

![ Output for AA Transmite](https://github.com/PEG-IITDH/lab-6-blinky-HarshitTak/blob/main/Desktop/Lab_8_UART/IMAGES/AA_transmit.png)

### Output for F0 Receive<br>
![ Output for F0 Receive](https://github.com/PEG-IITDH/lab-6-blinky-HarshitTak/blob/main/Desktop/Lab_8_UART/IMAGES/F0_recieve.png)

### Output for AA Receive<br>
![Output for AA Receive](https://github.com/PEG-IITDH/lab-6-blinky-HarshitTak/blob/main/Desktop/Lab_8_UART/IMAGES/AA_receive.png)



