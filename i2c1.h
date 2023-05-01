/* 
 * File:   i2c1.h
 * Author: Andy Page
 * Comments: Provides setup and read/write functions for I2C1
 * Revision history: 1, 26th October 2019
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INC_I2C1_H
#define	INC_I2C1_H

#include <xc.h> // include processor files - each processor file is guarded.  

extern unsigned char i2cFault;

void I2C1_Initialize(const unsigned long);
void I2C1_Start(void);
void I2C1_Repeated_Start(void);
unsigned char I2C1_Write_Byte_Read_Ack(unsigned char);
void I2C1_Stop(void);
unsigned char I2C1_Read_Byte(unsigned char);
void I2C1_Wait(void);
//unsigned char I2C1_CheckAck(void);
void I2C1_Check_Data_Stuck(void);


#endif	/* INC_I2C1_H */

