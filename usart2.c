#include "usart2.h"
#include "config.h"
#include <stdint.h>

//Configures serial port 2 8-bit
/**
 * Configures USART2 for serial port use with the defined baud rate.
 * @param baudrate   Baud rate - see defines
 */
void USART2_Start(const uint8_t baudrate){
    /***Set up USART1 for RS485 port **************/

    TXSTA2bits.CSRC  = 0;      //
    TXSTA2bits.TX9   = 0;      //Selects 8-bit transmission
    TXSTA2bits.TXEN  = 1;      //Transmit is enabled
    TXSTA2bits.SYNC  = 0;      //Asynchronous mode
    TXSTA2bits.SENDB = 0;      //don't Send Sync Break on next transmission (cleared by hardware upon completion)
    TXSTA2bits.BRGH  = 1;      //High baud rate

    /***RX Configuration************/
    RCSTA2bits.SPEN  = 1;      //Serial port is enabled
    RCSTA2bits.RX9   = 0;      //Selects 8-bit reception
    RCSTA2bits.CREN  = 1;      //Enables the receiver
    RCSTA2bits.ADDEN = 0;      //Disables address detection, all bytes are received and ninth bit can be used as parity bit

    /***BAUDRATE Configuration************/
    BAUDCON2bits.ABDOVF = 0;   //No BRG rollover has occurred
    BAUDCON2bits.DTRXP  = 0;   //Receive data (RXx) is not inverted (active-high) 0
    BAUDCON2bits.CKTXP  = 0;   //Idle state for transmit (TXx) is a high level
    BAUDCON2bits.BRG16  = 1;   //16-bit Baud Rate Generator SPBRGHx and SPBRGx used
    BAUDCON2bits.WUE    = 0;   //RXx pin is not monitored or the rising edge detected
    BAUDCON2bits.ABDEN  = 0;   //Baudrate Measurement (autobaud) is Disabled

    if(baudrate==BAUD_19200){
        SPBRGH2 = 0x03;
        SPBRG2 = 0x40;
        //This gives 19.2k baud rate (actually 19.21k +0.04%)
    }
    else if(baudrate==BAUD_9600){
        SPBRGH2 = 0x06;
        SPBRG2 = 0x82;
        //This gives 9200 baud rate (actually 9598.1 -0.02%)
    }
    else if(baudrate==BAUD_115200){
        SPBRGH2 = 0;
        SPBRG2 = 0x8A;
        //This gives 115200 baud rate (actually 115.11k -0.08%)
    }
    else{
        //Default is 57600
        SPBRGH2 =  0x01;    //EUSART1 Baud Rate Generator Register High Byte       (0)
        SPBRG2	= 0x17;    //EUSART1 Baud Rate Generator Register Low Byte (x) 0x15
        //This gives 57.6k baud rate (actually 57.55k -0.08%)
    }

//    PIR1bits.RC1IF=0; //Clear interrupt bit
//    PIE1bits.RC1IE=1; //Enable UART1 receive interrupt
//    INTCONbits.PEIE_GIEL=1; //Enable peripheral interrupts
//    INTCONbits.GIE_GIEH=1; //Enable global interrupts
}

/**
 * Puts a character into the transmit buffer of USART2 and waits for it to be transmitted.
 * @param data  The data byte to send.
 */
void putchar(char data){
  while(!TRMT2){
      
  }
  TXREG2 = data;
  //Wait here til transmit buffer is empty
  while(!TRMT2){
      
  }
}

/**
 * Puts a character into the transmit buffer of USART2 and waits for it to be transmitted.
 * @param data  The data byte to send.
 */
void putch(char data){
  while(!TRMT2){   
  }
  TXREG2 = data;
  //Wait here until transmit buffer is empty
  while(!TRMT2){   
  }
}

/**
 * Resets USART2, for example when there is a framing or overrun error.
 */
void USART2reset(){
    RCSTA2bits.SPEN  = 0;      //Serial port is disabled
    __delay_us(100);
    RCSTA2bits.SPEN = 1; //Serial port is enabled
}

/**
 * Disables the USART2 module to save power.
 */
void USART2_Stop(){
    RCSTA2bits.SPEN  = 0;      //Serial port is disabled
    RCSTA2bits.CREN  = 0;      //Disables the receiver
}



