/* 
 * File:   ADS1110.h
 * Author: Andy Page
 * Comments: Functions for reading and writing the ADS1110 16-bit A to D I.C.
 * Revision history: 1, 26th October 2019
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INC_ADS1110_H
#define	INC_ADS1110_H



#include <xc.h> // include processor files - each processor file is guarded.  

//I²C defines
#define ADS1110_ADDRESS 0b10010000  //I²C hardware address of ADS1110A0

/**
 * Writes a configuration byte to the ADS1110 device.
 * @param  Device hardware address.
 * @param  Configuration byte to write.
 */
void writeADS1110(unsigned char, unsigned char);

/**
 * Reads a 16-bit unsigned value from the specified ADS1110 device
 * @param  The device hardware address.
 * @return The 16-bit signed value read from the device.
 */
unsigned int readADS1110(unsigned char);

/**
 * Configures the ADS1110 as required.
 */
void configADS1110(void);

#endif	/* INC_ADS1110_H */

