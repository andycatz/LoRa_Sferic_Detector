/**
 * i2c1.c
 * Sets up I2C1
 * Also provides read and write functions.
 * Author: Andy Page
 * Version: 1, 26th October 2019
 */

#include <xc.h>
#include "defines.h"

//Sets up the i2c bus
void I2C1_Initialize(const unsigned long c){
    //Configure the pins of the PIC18F46K22 as per the datasheet for I2C operation
    TRISCbits.RC3=1;
    TRISCbits.RC4=1;
    ANSELCbits.ANSC3=0; //Turn off analog function on pin
    ANSELCbits.ANSC4=0; //Turn off analog function on pin
    SSP1CON1 = 0b00101000; //SSP Module as master
    
    // Set up baud rate to be 100kHz
    // Baud Rate Generator Value: SSP1ADD 159;  
    SSP1ADD = (_XTAL_FREQ/(4*c))-1; //(9F)
}

void I2C1_Check_Data_Stuck(void){
    unsigned char i=0;
    if(PORTCbits.RC4==0){
        //Data pin is stuck
        TRISCbits.RC3=0; //Set as output
        //Toggle clock 8 times to release data line
        for(i=0;i<8;i++){

            LATCbits.LATC3=0; //(PORTCbits.RC3 Did work to unlatch)
            __delay_ms(1);
            LATCbits.LATC3=1; //(PORTCbits.RC3 did work to unlatch)
            __delay_ms(1);
        }
        TRISCbits.RC3=1; //Set as input
    }
}

void I2C1_Wait(){
    unsigned char tries=0;
    while (((SSP1STAT & 0x04)||(SSP1CON2 & 0x1F))&&(tries<150)){
        tries++; //Timeout to escape from while loop
        
    } //I2C function in progress
}

//Sends a start condition on the I2C bus
void I2C1_Start(void){
    I2C1_Wait();
    SSP1CON2bits.SEN1=1;
}

/**
 * Sends a repeated start condition on the I2C bus
 */
void I2C1_Repeated_Start(void){
    I2C1_Wait();
    SSP1CON2bits.RSEN1=1; //Send repeated start condition
}

//Write a byte and return the acknowledge bit (good for checking if a device is present)
unsigned char I2C1_Write_Byte_Read_Ack(unsigned char d){
    I2C1_Wait();
    PIR1bits.SSP1IF=0; //Clear interrupt flag first otherwise the bit may already be set
    SSP1BUF=d; //Put the data in the transmit buffer (it will begin sending immediately)
    unsigned char tries=0;
    while(!PIR1bits.SSP1IF && tries<150){
        //Wait for transmission to complete and ack bit to be read back or timeout
        tries++;
    }
    return SSP1CON2bits.ACKSTAT1; //Read the acknowledge bit and return it.  0 is correct, 1 is not acknowledged.
}

void I2C1_Stop(){
    I2C1_Wait();
    SSP1CON2bits.PEN1=1;
}

//Receives a byte
unsigned char I2C1_Read_Byte(unsigned char a){
    unsigned char temp;
    I2C1_Wait();
    RCEN1 = 1;
    I2C1_Wait();
    temp = SSPBUF;      //Read data from SSPBUF
    I2C1_Wait();
    ACKDT1 = (a)?0:1;    //Acknowledge bit
    ACKEN1 = 1;          //Acknowledge sequence
    return temp;
}





