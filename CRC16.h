/* 
 * File:   CRC16.h
 * Author: Andy Page
 * Comments: Calculates a CRC16 for a byte sequence
 * Revision history: 1, 26th October 2019
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INC_CRC16_H
#define	INC_CRC16_H

#include <xc.h> // include processor files - each processor file is guarded.  

unsigned short int CRC16 (const unsigned char *, unsigned short int);

#endif	/* INC_CRC16_H */