/*
 * SPI.c
 *
 *  Created on: Jan 15, 2019
 *      Author: farshid
 */
#include <driverlib.h>
#include <msp430.h>


void spi_init() {
    //Initialize slave to MSB first, inactive high clock polarity and 3 wire SPI
    EUSCI_B_SPI_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
    param.clockSourceFrequency = CS_getSMCLK();
    param.desiredSpiClock = 1000000;    //4MHz
    param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    param.spiMode = EUSCI_B_SPI_3PIN;
    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &param);


    //  //Enable SPI Module
//    UCB0BR0 = 0x01;                           // /2,fBitClock = fBRCLK/(UCBRx+1).
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);
    //  //Enable Receive interrupt
    EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT | EUSCI_B_SPI_TRANSMIT_INTERRUPT);


}
