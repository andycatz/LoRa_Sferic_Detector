# LoRa_Sferic_Detector
Code for LoRa Sferic Detector.  PIC18F46K22 written in C using Microchip XC8 compiler.
Created on 11 September 2021, 21:06
 * 
 * Version 1
 * 
 * Version 2 - 27th Sept 2021 - Changed to 50 byte common data format.
 * Version 3 - 27th Dec 2021 - Added second sferic sensor input on RB4.
 * 
 * LoRa Lightning Sferic Sensor Transmitter.
 * 
 * Sferics sensed using lightning detector modules connected to RB5 & RB4.
 * 
 * RFM95W LoRa Module

 * AN0 reads battery voltage through a resistor divider
 * AN1 reads local temperature through 10k NTC and 10k resistor as a divider from 3.3V
 * AN2 reads VIN
 * AN3 reads Sferic Detector Module Supply Voltage
 * RE2 has red LED
 * RE1 has green LED
 * RB5 has sferic pulse input.
 * RB4 has sferic pulse input from sensor at 90 degrees to RB5.
 * 
 * Internal oscillator used at 64MHz (16MHz with 4x PLL)
