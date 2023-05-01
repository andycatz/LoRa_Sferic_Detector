#include <xc.h>
#include "LoRa.h"
#include <stdint.h>
#include <stdio.h>

#define DEBUG 0

/**
 * Configures PIC and LoRa module RFM95W to start with specified frequency in MHz
 * from PIC18F46K22_LoRA_UVVIS_V2
 */
void LoRaStart(float freq, uint8_t syncWord){
    //Configure pin for LoRa module reset
    ANSELAbits.ANSA2=0; //Digital input buffer enabled
    
    
    //Configure SPI2 as master
    //Set up SPI pins first
    TRISDbits.RD1=1; //SDIx must have corresponding TRIS bit set (input)
    TRISDbits.RD4=0; //SDOx must have corresponding TRIS bit cleared (output)
    TRISDbits.RD0=0; //SCKx (Master mode) must have corresponding TRIS bit cleared (output)
    TRISDbits.RD3=0; //#SS must have corresponding TRIS bit cleared (output)
    ANSELDbits.ANSD1=0; //Input buffer enabled
    ANSELDbits.ANSD4=0; //Digital
    ANSELDbits.ANSD3=0; //Digital
    ANSELDbits.ANSD0=0; //Digital
    LATDbits.LATD3=1; //Set SS high so chip is not selected
    
    //Clock polarity
    SSP2CON1bits.CKP=0; //Clock idle low, active high
    SSP2STATbits.CKE=1; //Active to idle 1
    
    //Input data sampling
    SSP2STATbits.SMP=1; //Input data sampled at end of data output time 1
    
    //SPI Mode and clock
    SSP2CON1bits.SSPM=0b0010; //SPI Master Mode, clock = Fosc/64
    
    //SPI Enable
    SSP2CON1bits.SSPEN=1; //Enabled
    
    LoRaReset();
    __delay_ms(10);
    setLoRaMode();
    __delay_ms(10);
    LoRaOptimalLoad(syncWord);
    LoRaSetFrequency(freq); //Can only set in standby or sleep modes
}

uint8_t LoRaGetVersion(){
    uint8_t temp = SPI2ReadByte(VERSION_REG);
    return temp;
}

void LoRaReset(){
    //Perform reset
    TRISCbits.RC6=0; //Configure port as an output
    LATCbits.LATC6=0; //Set low
    __delay_ms(1);
    TRISCbits.RC6=1; //Configure port as input (goes high-Z)
    __delay_ms(5);
}

void setLoRaMode(){
    //Set Long Range Mode bit in OpMode register
    uint8_t regValue = readOpModeRegister();
    regValue = regValue | LORA_MODE; //Set bit 7 high and leave others as is
    writeOpModeRegister(regValue); //Write the value back
}

uint8_t readOpModeRegister(){
    return SPI2ReadByte(OP_MODE_REG);
}

void writeOpModeRegister(uint8_t regValue){
    SPI2WriteByte(OP_MODE_REG, regValue);
}



void LoRaMode_RXActive(){
    SPI2WriteByte(OP_MODE_REG, 0b1000101); //LoRa Mode with Receiver active all the time
}

/**
 * SPI2WriteByte
 * Writes an address byte, then writes a data byte
 * @param address
 * @param data
 */
void SPI2WriteByte(uint8_t address, uint8_t data){
    SSP2IF=0; //Clear interrupt flag
    LATDbits.LATD3=0; //Set SS low
    __delay_us(5);
    address = address|0x80; //bit 7 set to indicate a register write
    SSP2BUF=address; //Write address to SPI buffer 
    while(!SSP2IF){
        //Wait for transmission to complete
    }
    SSP2IF=0; //Clear interrupt flag
    __delay_us(5);
    SSP2BUF=data; 
    while(!SSP2IF){
        //Wait for transmission and reception to complete
    }
    __delay_us(5);
    LATDbits.LATD3=1; //Set SS high
    SSP2IF=0; //Clear interrupt flag
    uint8_t dataByte = SSP2BUF; //A byte has been received but this is not used.
}

/**
 * SPI2ReadByte
 * Writes an address byte then reads a byte back.  0 is sent as a dummy value
 * for the second transfer.
 * @param address
 * @return 
 */
uint8_t SPI2ReadByte(uint8_t address){
    SSP2IF=0; //Clear interrupt flag
    LATDbits.LATD3=0; //Set SS low
    SSP2BUF=address; //Write data to SPI buffer
    while(!SSP2IF){
        //Wait for transmission to complete
    }
    SSP2IF=0; //Clear interrupt flag
    SSP2BUF=0; //Data byte - for a read, set to 0
    while(!SSP2IF){
        //Wait for transmission and reception to complete
    }
    SSP2IF=0; //Clear interrupt flag
    //__delay_us(20);
    LATDbits.LATD3=1; //Set SS high
    uint8_t dataByte = SSP2BUF;
    return dataByte;
}

/* 
 * Transmits a data packet.
 */
void LoRaTXData(uint8_t* data, uint8_t dataLength){
    //Must be in standby mode for this to work
    LoRaStandbyMode();
    if(DEBUG){
        printf("Transmitting.... ");
    }
    SPI2WriteByte(FIFO_ADD_PTR_REG, 0);
    SPI2WriteByte(PAYLOAD_LENGTH_REG, 0);
    

    for(uint8_t i=0;i<dataLength;i++){
        SPI2WriteByte(FIFO_REG, data[i]);
    }
    SPI2WriteByte(PAYLOAD_LENGTH_REG, dataLength);
    LoRaTXMode(); //Set TX mode to send the message
    if(DEBUG){
        printf("Done.\r\n");
    }
    //Will return to standby mode automatically when finished.
    //You can check TxDone interrupt to see if it's finished.
}

/**
 * Sets the LoRa module into standby mode
 */
void LoRaStandbyMode(){
    uint8_t regValue = readOpModeRegister(); //Read whats in there already
    regValue = regValue & 0b11111000; //Blank out other modes
    regValue = regValue | STANDBY_MODE; //Set bit 0 high and leave others as is
    writeOpModeRegister(regValue); //Write the value back
}

void LoRaSleepMode(){
    uint8_t regValue = readOpModeRegister(); //Read whats in there already
    regValue = regValue & 0b11111000; //Blank out other modes
    regValue = regValue | SLEEP_MODE;
    writeOpModeRegister(regValue); //Write the value back
}

void LoRaFreqSynthRXMode(){
    uint8_t regValue = readOpModeRegister(); //Read whats in there already
    regValue = regValue & 0b11111000; //Blank out other modes
    regValue = regValue | FREQ_SYNTH_RX_MODE; //Set bit 0 high and leave others as is
    writeOpModeRegister(regValue); //Write the value back
}

void LoRaFreqSynthTXMode(){
    uint8_t regValue = readOpModeRegister(); //Read whats in there already
    regValue = regValue & 0b11111000; //Blank out other modes
    regValue = regValue | FREQ_SYNTH_TX_MODE; //Set bit 0 high and leave others as is
    writeOpModeRegister(regValue); //Write the value back
}

void LoRaTXMode(){
    if(DEBUG){
        printf("TX Mode\r\n");
    }
    uint8_t regValue = readOpModeRegister(); //Read whats in there already
    regValue = regValue & 0b11111000; //Blank out other modes
    regValue = regValue | TX_MODE; //Set bit 0 high and leave others as is
    writeOpModeRegister(regValue); //Write the value back
}

void LoRaRXContinuousMode(){
    uint8_t regValue = readOpModeRegister(); //Read whats in there already
    regValue = regValue & 0b11111000; //Blank out other modes
    regValue = regValue | RX_CONT_MODE; //Set bit 0 high and leave others as is
    writeOpModeRegister(regValue); //Write the value back02
}

/**
 * Sets the frequency of the LoRa module
 * @param  Frequency in MHz
 * 
 * Frf = XOSC * FreqReg/2^19
 * Resolution is 61.035Hz for XOSC=32MHz.
 */
void LoRaSetFrequency(float freqMHz){
    uint32_t intermediate = (freqMHz*16384);
    //printf("Intermediate %lu\r\n",intermediate);
    uint8_t msb = (intermediate>>16) & 0xFF; //Extract MSB
    uint8_t mid = (intermediate>>8)& 0xFF; //Extract mid byte
    uint8_t lsb = intermediate & 0xFF; //Extract LSB
    //printf("MSB %d, MID %d, LSB %d\r\n",msb,mid,lsb);
    SPI2WriteByte(FRF_MSB_REG,msb);
    SPI2WriteByte(FRF_MID_REG,mid);
    SPI2WriteByte(FRF_LSB_REG,lsb);
}

/**
 * Gets the centre frequency from the device.
 * @return Frequency in MHz.
 */
float LoRaGetFrequency(){
    uint8_t msb = SPI2ReadByte(FRF_MSB_REG);
    uint8_t mid = SPI2ReadByte(FRF_MID_REG);
    uint8_t lsb = SPI2ReadByte(FRF_LSB_REG);
    uint32_t intermediate = (uint32_t)msb<<16 | (uint32_t)mid<<8 | lsb;
    float freqMHz = (float)intermediate/16384.0;
    return freqMHz;
}


uint8_t LoRaGetIRQFlags(){
    uint8_t regValue = SPI2ReadByte(IRQ_FLAGS_REG);
    return regValue;
}

void LoRaClearIRQFlags(){
    SPI2WriteByte(IRQ_FLAGS_REG,0xFF);
}


/**
 * Dumps the contents of all registers to printf
 */
void LoRaDumpRegisters(){
    for(uint8_t reg=0x0;reg<0x20;reg++){
        printf("Reg %X:%X\r\n", reg, SPI2ReadByte(reg));
    }
}

/**
 * Loads all the registers required to setup an optimal configuration
 */
void LoRaOptimalLoad(uint8_t syncWord){
    LoRaSleepMode(); //Can only change to LoRa mode in sleep mode
    setLoRaMode();
    LoRaStandbyMode();
    __delay_ms(10); //Need a delay to come up to standby mode
    SPI2WriteByte(0x06, 0xD9);
    SPI2WriteByte(0x07, 0);
    SPI2WriteByte(0x08, 0);
    SPI2WriteByte(0x09, 0x8F);
    SPI2WriteByte(0x0A, 0x09);
    SPI2WriteByte(0x0B, 0x2B);
    SPI2WriteByte(0x0C, 0x23);
    SPI2WriteByte(0x0E, 0);
    SPI2WriteByte(0x0F, 0);
    SPI2WriteByte(0x10, 0);
    SPI2WriteByte(0x11, 0);
    SPI2WriteByte(0x13, 0);
    SPI2WriteByte(0x1D, 0x72);
    SPI2WriteByte(0x1E, 0x70);
    SPI2WriteByte(0x1F, 0x64);
    SPI2WriteByte(0x20, 0);
    SPI2WriteByte(0x21, 0x08);
    SPI2WriteByte(0x23, 0xFF);
    SPI2WriteByte(0x24, 0);
    SPI2WriteByte(0x25, 0);
    SPI2WriteByte(0x26, 0x04);
    SPI2WriteByte(0x2F, 0x45);
    SPI2WriteByte(0x30, 0x55);
    SPI2WriteByte(0x31, 0xC3);
    SPI2WriteByte(0x33, 0x27);
    SPI2WriteByte(0x36, 0x03);
    SPI2WriteByte(0x37, 0x0A);
    SPI2WriteByte(0x39, syncWord); //Sync word was 0x12
    SPI2WriteByte(0x3A, 0x49);
    SPI2WriteByte(0x4B, 0x09);
    SPI2WriteByte(0x4D, 0x84);
    SPI2WriteByte(0x61, 0x1C);
    SPI2WriteByte(0x62, 0x0E);
    SPI2WriteByte(0x63, 0x5B);
    SPI2WriteByte(0x64, 0xCC);
    SPI2WriteByte(0x70, 0xD0);
}


