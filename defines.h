/* Author: Andy Page
 * Comments: defines.h for MODBUS_TH program
 * Revision history: 1, 26th October 2019
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INC_DEFINES_H
#define	INC_DEFINES_H

#define _XTAL_FREQ 64000000
#define GREEN_LED LATEbits.LATE1 //Green LED output port
#define RED_LED LATEbits.LATE2 //Red LED output port


#endif	/* INC_DEFINES_H */