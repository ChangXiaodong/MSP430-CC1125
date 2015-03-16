#include "hal_types.h"
#include "msp430.h"
uint8 UartRxGetByte(void)
{
    while(!(IFG2 & UCA0RXIFG));    // Wait for TX buffer ready to receive new byte
    
    return UCA0RXBUF;

}

void Uart_init()
{

    // For the moment, this UART implementation only
    // supports communication settings 115200 8N1
    // i.e. ignore baudrate and options arguments.

    UCA0CTL1 |= UCSWRST;                   // Keep USART1 in reset state

    UCA0CTL1 |= UCSSEL1;                  // Set clock source SMCLK
    UCA0CTL1 &= ~UCSSEL0;

    P3SEL |= BIT4;                    // P3.4 = USART1 TXD
    P3SEL |= BIT5;                    // P3.5 = USART1 RXD


    UCA0BR0 = 0x08;                     // 1MHz 115200 »ò 0x09
    UCA0BR1 = 0x00;                     // 1MHz 115200

  

    UCA0CTL0 &= ~UCPEN;                   // No parity
    UCA0CTL0 &= ~UCSPB;                   // 1 stop bit
    UCA0CTL0 &= ~UC7BIT;                  // 8 data bits

    UCA0CTL1 &= ~UCSWRST;                   // Initialize USART1 state machine

    // Enable RX interrupt
    //halUartRxIntEnable();

    // Set RTS pin to output
    //HAL_RTS_DIR_OUT();
    // Enable RX Flow
    //halUartEnableRxFlow(TRUE);


}
uint8 Uart_Write(uint8* buf, uint8 length)
{

    uint8 i;
    for(i = 0; i < length; i++)
    {

        while(!(IFG2 & UCA0TXIFG));    // Wait for TX buffer ready to receive new byte
        UCA0TXBUF = buf[i];            // Output character
    }
    return (i+1);

}