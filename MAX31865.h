/* 
 * File:   MAX31865.h
 * Author: Andy Page
 * Comments: Functions for reading and writing the MAX31865 PT interface I.C.
 * Revision history: 1, 26th October 2019
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INC_MAX31865_H
#define	INC_MAX31865_H

//Configuration:
//D7    Vbias                   1=ON
//D6    Conversion Mode         1=Auto, 0=Normally off
//D5    1-shot                  1=1 shot
//D4    1=3-wire                0=2 or 4-wire
//D3    Fault Detection Cycle Control
//D2    Fault Detection Cycle Control
//D1    Fault Status Clear      1=Clear
//D0    50/60Hz filter select   1=50Hz 0=60Hz
#define MAX31865_CONFIG 0b11000001; 

#define SPICS LATCbits.LATC0        //Pin used as SPI chip select
#define WRITE_CONFIG 0x80
#define WRITE_HIGH_FAULT_THR_MSB 0x83
#define WRITE_HIGH_FAULT_THR_LSB 0x84
#define WRITE_LOW_FAULT_THR_MSB 0x85
#define WRITE_LOW_FAULT_THR_LSB 0x86
#define READ_CONFIG 0
#define READ_MSB 1
#define READ_LSB 2

#include <xc.h> // include processor files - each processor file is guarded.  

/**
 * Writes a byte to the device
 * @param       Register address
 * @param       Data to write
 */
void MAX31865writeByte(unsigned char, unsigned char);

/**
 * Writes 2 bytes of data to the specified address in the device.
 * @param regAddress
 * @param data First byte to write (MSB)
 * @param data2 Second byte to write (LSB)
 */
void MAX31865write2Bytes(unsigned char, unsigned char, unsigned char);

/**
 * Reads a byte from the device
 * @param   Register address
 * @return  Byte read from device
 */
//unsigned char MAX31865readByte(unsigned char);

/**
 * Writes to the configuration register of the device.
 * @param config
 */
void MAX31865writeConfigByte(unsigned char);

/**
 * Reads a data value from the device (raw data)
 * @return 
 */
uint16_t MAX31865readData();


/**
 * Configures the SPI module for this device.
 */
void configureSPI(void);

///**
// * Reads the current temperature from the device.
// */
//uint16_t readTemperature(void);

/**
 * Writes a value to the high fault register of the MAX31865
 * @param v  16-bit unsigned value to write to the register.
 */
//void writeTempHighFaultRegister();

/**
 * Writes a value to the low fault register of the MAX31865
 * @param v  16-bit unsigned value to write to the register.
 */
//void writeTempLowFaultRegister();


#endif	/* INC_MAX31865_H */

