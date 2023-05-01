/* 
 * File:   lora.h
 * Author: Andy Page
 * Comments: LoRa library for RFM95W module used with PIC18F46K22 on SPI2 port
 * Revision history: Version 1, 26th October 2019 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef LORA_H
#define	LORA_H

#include <stdint.h>

#define _XTAL_FREQ 64000000
//NB ONLY LoRa REGISTERS ARE DEFINED - FSK/OOK MODE REGISTERS ARE NOT!
#define FIFO_REG 0x00
#define OP_MODE_REG 0x01
#define FRF_MSB_REG 0x06
#define FRF_MID_REG 0x07
#define FRF_LSB_REG 0x08
#define PA_CONFIG_REG 0x09
#define PA_RAMP_REG 0x0A
#define OCP_REG 0x0B
#define LNA_REG 0x0C
#define FIFO_ADD_PTR_REG 0x0D
#define FIFO_TX_BASE_ADDR_REG 0x0E
#define FIFO_RX_BASE_ADDR_REG 0x0F
#define FIFO_RX_CURRENT_REG 0x10
#define IRQ_FLAGS_MASK_REG 0x11
#define IRQ_FLAGS_REG 0x12
#define RX_NB_BYTES_REG 0x13
#define RX_HEADER_CNT_VALUE_MSB_REG 0x14
#define RX_HEADER_CNT_VALUE_LSB_REG 0x15
#define RX_PACKET_CNT_VALUE_MSB_REG 0x16
#define RX_PACKET_CNT_VALUE_LSB_REG 0x17
#define MODEM_STAT_REG 0x18
#define PKT_SNR_VALUE_REG 0x19
#define PKT_RSSI_VALUE_REG 0x1A
#define RSSI_VALUE_REG 0x1B
#define HOP_CHANNEL_REG 0x1C
#define MODEM_CONFIG_1_REG 0x1D
#define MODEM_CONFIG_2_REG 0x1E
#define SYMB_TIMEOUT_LSB_REG 0x1F
#define PREAMBLE_MSB_REG 0x20
#define PREAMBLE_LSB_REG 0x21
#define PAYLOAD_LENGTH_REG 0x22
#define MAX_PAYLOAD_LENGTH_REG 0x23
#define HOP_PERIOD_REG 0x24
#define FIFO_RX_BYTE_ADDR_REG 0x25
#define MODEM_CONFIG_3_REG 0x26
#define SYNC_VALUE_REG 0x39
//0x27 to 0x3F RESERVED in LoRa mode (0x3C, 0x3D???)
#define TEMP_REG 0x3C
#define LOW_BAT_REG 0x3D
#define DIO_MAPPING_1_REG 0x40
#define DIO_MAPPING_2_REG 0x41
#define VERSION_REG 0x42
//0x43, 0x44 UNUSED
#define TXCO_REG 0x4B
#define PA_DAC_REG 0x4D

#define FORMER_TEMP_REG 0x5B

#define AGC_REF_REG 0x61
#define AGC_THRESH_1_REG 0x62
#define AGC_THRESH_2_REG 0x63
#define AGC_THRESH_3_REG 0x64


//Operating modes
#define STANDBY_MODE 0b00000001
#define SLEEP_MODE 0b00000000
#define FREQ_SYNTH_TX_MODE 0b00000010
#define TX_MODE 0b00000011
#define FREQ_SYNTH_RX_MODE 0b00000100
#define RX_CONT_MODE 0b00000101
#define RX_SINGLE_MODE 0b00000110
#define CAD_MODE 0b00000111
#define LORA_MODE 0b10000000

//Bandwidths to use with set and get bandwidth
#define BW7k8 0b0000
#define BW10k4 0b0001
#define BW15k6 0b0010
#define BW20k8 0b0011
#define BW31k25 0b0100
#define BW41k7 0b0101
#define BW62k5 0b0110
#define BW125k 0b0111
#define BW250k 0b1000
#define BW500k 0b1001



void LoRaStart(float, uint8_t);
uint8_t LoRaGetVersion();
void LoRaReset();
void setLoRaMode(); //Sets module into LoRa mode
uint8_t readOpModeRegister();
void writeOpModeRegister(uint8_t);

void LoRaSleepMode(); //Set sleep mode
void LoRaStandbyMode(); //Set standby mode
void LoRaFreqSynthRXMode();
void LoRaFreqSynthTXMode();
void LoRaTXMode();
void LoRaRXContinuousMode();
void LoRaMode_RXActive(); //Set LoRa mode with receiver always active

//int8_t LoRaGetTemp();
void LoRaTXData(uint8_t* , uint8_t); //Sends a data packet of length dataLength
//void LoRaSetPreamble(uint16_t);
//uint16_t LoRaGetPreamble();
void SPI2WriteByte(uint8_t, uint8_t);
uint8_t SPI2ReadByte(uint8_t);
void LoRaSetFrequency(float);
float LoRaGetFrequency(void);
//void LoRaSetBandwidth(uint8_t);
//uint8_t LoRaGetBandwidth();
uint8_t LoRaGetIRQFlags();
void LoRaClearIRQFlags();
//void LoRaSetPAConfig(uint8_t);
//uint8_t LoRaGetPAConfig();
//void LoRaSetPABoostOn();
//uint8_t LoRaGetPABoostOn();
//void LoRaSetOutputPower(uint8_t);
//uint8_t LoRaGetOutputPower();
//void LoRaSetMaxOutputPower(uint8_t);
//uint8_t LoRaGetMaxOutputPower();
//void LoRaSetOCPTrim(uint8_t);
//uint8_t LoRaGetOCPTrim();
//void LoRaSetOCPOnOff(uint8_t);
//void LoRaSetCodingRate(uint8_t);
//uint8_t LoRaGetCodingRate();
//void LoRaImplicitHeaderMode();
//void LoRaExplicitHeaderMode();
//void LoRaSetSpreadingFactor(uint8_t);
//uint8_t LoRaGetSpreadingFactor();
//uint8_t LoRaGetSyncWord();
//void LoRaSetSyncWord(uint8_t);
//void LoRaSetPayloadLength(uint8_t);
//void LoRaSetMaxPayloadLength(uint8_t);
//uint8_t LoRaGetPayloadLength();
//uint8_t LoRaGetMaxPayloadLength();
void LoRaDumpRegisters();
void LoRaOptimalLoad(uint8_t); //Provides an optimal register load to get working quickly.


#endif	/* CONFIG_H */
