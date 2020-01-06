/*
 * uart.c
 *
 *  Created on: Jan 18, 2019
 *      Author: farshid
 */
#include <driverlib.h>
#include <msp430.h>

void uart_init() {



    // Configuration for 115200 UART with SMCLK at 16384000
    // These values were generated using the online tool available at:
    // http://software-dl.ti.com/msp430/msp430 public sw/mcu/msp430/MSP430BaudRateConverter/index.html
//    EUSCI_A_UART_initParam uartConfig = {
//    EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
//    1, // BRDIV = 8  clockPrescalar
//    0, // UCxBRF = 14  firstModReg
//    0xF7, // UCxBRS = 34=0x22  secondModReg
//    EUSCI_A_UART_NO_PARITY, // No Parity
//    EUSCI_A_UART_LSB_FIRST, // MSB First
//    EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
//    EUSCI_A_UART_MODE, // UART mode
//    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling Baudrate
//    };

    // Configure UART
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;

    // Baud Rate calculation : 460800
    // 16000000/(16*460800) = 2.17013
    // Fractional portion = 0.083
    // User's Guide Table 22-5: UCBRSx = 0x49
    // UCBRFx = int ( (2.17013-2)*16) = 2
    UCA0BR0 = 2;                             // 16000000/16/460800
    UCA0BR1 = 0x00;
    UCA0MCTLW = 0xBB00 | UCOS16 | UCBRF_2;

    // Settings P2.0 and P2.1 as UART pins.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
    GPIO_PIN0 | GPIO_PIN1,
    GPIO_PRIMARY_MODULE_FUNCTION);

    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
//    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt


//    // Configure and enable the UART peripheral
//    EUSCI_A_UART_init(EUSCI_A0_BASE, &uartConfig);
//    EUSCI_A_UART_enable(EUSCI_A0_BASE);
}
