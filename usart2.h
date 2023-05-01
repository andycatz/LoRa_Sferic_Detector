/* 
 * File:   usart2.h
 * Author: Andy Page
 * Comments: sets up usart2
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef INC_USART2_H
#define	INC_USART2_H

#define BAUD_9600 0
#define BAUD_19200 1
#define BAUD_57600 2
#define BAUD_115200 3

#include <xc.h> // include processor files - each processor file is guarded. 
#include <stdint.h>

void USART2_Start(const uint8_t);

void putchar(char);

void USART2reset(void);

void USART2_Stop(void);

#endif	/* INC_USART2_H */

