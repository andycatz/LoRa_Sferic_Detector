/**
 * ADS1110.c
 * Functions to read and write the ADS1110 16-bit A to D Converter I.C.
 */

#include <xc.h>
#include "i2c1.h"
#include "ADS1110.h"
#include <stdint.h>

unsigned char i2cFault=0;

/**
 * Writes the configuration byte to the device
 * @param address
 * @param config
 */
void writeADS1110(unsigned char address, unsigned char config){
    I2C1_Start();
    i2cFault = I2C1_Write_Byte_Read_Ack(address);
    i2cFault = I2C1_Write_Byte_Read_Ack(config);
    I2C1_Stop();
}

/**
 * Reads the 16-bit value from the device
 * @param address   Hardware I²C address
 * @return          16-bit signed data from device
 */
unsigned int readADS1110(unsigned char address){
    I2C1_Start();
    i2cFault = I2C1_Write_Byte_Read_Ack(address+1u);
    char msb = I2C1_Read_Byte(1); //Read byte and send acknowledge
    char lsb = I2C1_Read_Byte(1); //Read byte and do not send acknowledge
    char cfg = I2C1_Read_Byte(0); //Read config byte (not used))
    I2C1_Stop();
    return msb*256u+lsb; //Calculate the result;
}

void configADS1110(){
    writeADS1110(ADS1110_ADDRESS,0x0C); //Default configuration (above should initialize it)
}
