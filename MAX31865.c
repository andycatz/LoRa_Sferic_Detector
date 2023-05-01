/**
 * Functions for reading and writing the MAX31865 RTC interface I.C.
 * 
 * SCLK will operate from DC to 5MHz on this device.
 * This device supports SPI modes 1 & 3.
 * 
 *
 * Created on 19 October 2019, 20:27
 * 
 * MAX31865 Registers (Add 80h to write to them)
 * Register Name            Address (hex)
 * Configuration            00
 * RTD MSBs                 01
 * RTD LSBs                 02
 * High Fault Threshold MSB 03
 * High Fault Threshold LSB 04
 * Low Fault Threshold MSB  05
 * Low Fault Threshold LSB  06
 * Fault Status             07
 */

#include <xc.h>
#include <stdint.h>
#include "MAX31865.h"
#include "defines.h"
#include <stdio.h>
#include <stdint.h>

uint8_t temp_config_byte=0;
uint8_t tempStatus=0;
uint8_t tempStatusBit=0;
uint8_t tempHighFault = 0;
uint8_t tempLowFault=0;


/**
 * Writes a byte of data to the specified address in the device.
 * @param regAddress
 * @param data
 */
void MAX31865writeByte(unsigned char regAddress, unsigned char data){
    unsigned int tries=0;
    PIR3bits.SSP2IF=0;//Clear interrupt flag
    SPICS=0; //Set chip select low
    SSP2BUF=regAddress; //Write the address
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    tries=0;
    SSP2BUF=data; //Write the data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    SPICS=1; //Set chip select high
}

/**
 * Writes 2 bytes of data to the specified address in the device.
 * @param regAddress
 * @param data First byte to write (MSB)
 * @param data2 Second byte to write (LSB)
 */
void MAX31865write2Bytes(unsigned char regAddress, unsigned char data, unsigned char data2){
    unsigned int tries=0;
    PIR3bits.SSP2IF=0;//Clear interrupt flag
    SPICS=0; //Set chip select low
    SSP2BUF=regAddress; //Write the address
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    tries=0;
    SSP2BUF=data; //Write the first byte of data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    
    tries=0;
    SSP2BUF=data2; //Write the second byte of data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    
    SPICS=1; //Set chip select high
}

/**
 * Writes the specified value to the configuration register
 * @param config  The value to write to the configuration register
 */
void MAX31865writeConfigByte(unsigned char config){
    MAX31865writeByte(WRITE_CONFIG, config);
}

/**
 * Reads temperature data from device.
 * @return 
 */
uint16_t MAX31865readData(){
    unsigned int tries=0;
    PIR3bits.SSP2IF=0;//Clear interrupt flag
    SPICS=0; //Set chip select low
    SSP2BUF=0; //Write the address
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout so we can escape from while loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    if(tries>49){
        printf("No MAX31865!");
    }
    
    SSP2BUF=0; //Dummy write data
    tries=0;
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++;
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    temp_config_byte = SSP2BUF; //Data available
    
    SSP2BUF=0; //Dummy write data
    tries=0;
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++;
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag
    tries=0;
    uint8_t MSB = SSP2BUF; //Data available
    printf("MSB %d\r\n", MSB);
    
    SSP2BUF=0; //Dummy write data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag

    uint8_t LSB = SSP2BUF; //Data available
    printf("LSB %d\r\n", LSB);
    
    tries=0;
    SSP2BUF=0; //Dummy write data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag

    unsigned char HFTMSB= SSP2BUF; //Data available
    printf("HFTMSB %d\r\n", HFTMSB);
    
    tries=0;
    SSP2BUF=0; //Dummy write data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag

    unsigned char HFTLSB= SSP2BUF; //Data available
    printf("HFTMSB %d\r\n", HFTLSB);
    
    tries=0;
    SSP2BUF=0; //Dummy write data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag

    unsigned char LFTMSB= SSP2BUF; //Data available
    printf("LFTMSB %d\r\n", LFTMSB);
    
    tries=0;
    SSP2BUF=0; //Dummy write data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag

    unsigned char LFTLSB= SSP2BUF; //Data available
    printf("LFTLSB %d\r\n", LFTLSB);
    
    tries=0;
    SSP2BUF=0; //Dummy write data
    //Wait for byte to send
    while(!PIR3bits.SSP2IF && tries<50){
        tries++; //Simple timeout to escape from loop
    }
    PIR3bits.SSP2IF=0; //Clear interrupt flag

    tempStatus = SSP2BUF; //Data available
    printf("STATUS %d\r\n", tempStatus);
    
    SPICS=1; //Set chip select high
    
    tempStatusBit = LSB & 0x01; //Extract status bit
    
    tempHighFault = HFTMSB*256 + HFTLSB;
    
    tempLowFault = LFTMSB * 256 + LFTLSB;
    
    return (MSB*256 + LSB)/2; //Shift result down 1-bit as bit 0 is an error bit
}





/**
 * Configures the SPI2 module for this device.
 */
void configureSPI(void){
    SPICS=1; //Set chip select pin to 1 (not selected)
    SSP2STATbits.SMP=1; //Input data sample at end of data output time
    SSP2STATbits.CKE=0; //Transmit occurs on transition from active to idle clock state
    SSP2CON1bits.CKP=0; //Idle state for clock is a high level
    SSP2CON1bits.SSPM=0b0001; //SPI master mode, clock = Fosc/16 = (64/16=4MHz)
    SSP2CON1bits.SSPEN2=1; //Enable SPI2
}

///**
// * Reads the current temperature.
// */
//uint16_t readTemperature(void){
//    uint16_t tempTemp = MAX31865readData(); //Raw temperature data.  Reads all the other status stuff as well.
//    return tempTemp;
//}

///**
// * Writes a value to the high fault register of the MAX31865
// */
//void writeTempHighFaultRegister(){
//    unsigned int v = TEMP_HIGH_FAULT_REG; //Get value
//    unsigned char MSB = (v>>8)&0xFF;
//    unsigned char LSB = v & 0xFF;
//    MAX31865writeByte(WRITE_HIGH_FAULT_THR_MSB, MSB); //Address + 0x80 when writing to it.
//    MAX31865writeByte(WRITE_HIGH_FAULT_THR_LSB, LSB);
//}

///**
// * Writes a value to the low fault register of the MAX31865
// */
//void writeTempLowFaultRegister(){
//    unsigned int v = TEMP_LOW_FAULT_REG; //Get value
//    unsigned char MSB = (v>>8)&0xFF;
//    unsigned char LSB = v & 0xFF;
//    //MAX31865write2Bytes(0x05, MSB, LSB); //Write to 2 consecutive 8-bit registers in the temperature sensor device.
//    MAX31865writeByte(WRITE_LOW_FAULT_THR_MSB, MSB);
//    MAX31865writeByte(WRITE_LOW_FAULT_THR_LSB, LSB);
//}



