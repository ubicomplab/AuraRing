#include <msp430.h> 
#include <driverlib.h>
#include <SPI.h>
#include <timer.h>
#include <uart.h>
#include <clock.h>

#define BUFFER_SIZE 8       //Buffer size for LPF, only powers of two
const int SPITXData = 0x07; // dummy data to trigger SPI rx interrupt
uint16_t SPIRXData = 0x00;          //SPI 8 bit reading
uint32_t sampled = 0;       //6 channel adc readings
uint8_t counter_timer = 0;      //timer need to trigger 4 times for the whole adc read
uint8_t adc_channel = 0;
uint8_t buffer_count = 0;
int16_t adc_a[BUFFER_SIZE][3];
int16_t adc_b[BUFFER_SIZE][3];
int16_t filter[3];
uint16_t frame_num = 0;


long aclk =0;
long mclk =0;
long smclk =0;

void low_pass(int16_t channel_b[][3]) {
    int32_t sum_b[3] = {0,0,0};
    uint8_t i=0; uint8_t j=0;
    for (j=3; j > 0; j--) {
        for (i=BUFFER_SIZE; i > 0; i--) {
            sum_b[j-1] += channel_b[i-1][j-1];
        }
        filter[j-1] = sum_b[j-1] >> 3;

    }
}
/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
    SYSCFG3 = 1;                // To enble UART to be Port 2 instead of Port 1

//	// Setting P1.1(SCLK), P1.2(SIMO) and P1.3(SOMI) as SPI pins.
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION);

	// Setting P1.7 as ~CS for SPI
    P1DIR |= BIT7;
    P1OUT |= BIT7;

//     Setting P1.4, P1.5 ,P1.6 as A0A1A2: 000=1-2dif, 010=3-4dif, 001=5-6dif
    P1DIR |= BIT4 | BIT5 | BIT6;
    P1OUT &= ~BIT4 & ~BIT5 & ~BIT6;

    clock_init();

    aclk = CS_getACLK();
    smclk = CS_getSMCLK();
    mclk = CS_getMCLK();

    //Setting up TIMER_A
//    timer_init();

	spi_init();
	uart_init();
	PM5CTL0 &= ~LOCKLPM5;

	while (1) {
	    for (buffer_count=BUFFER_SIZE; buffer_count > 0; buffer_count--) {
            // Single SPI takes 72us,(1/5e06)*32*9
            for (adc_channel = 3; adc_channel > 0; adc_channel--)
            {
                switch (adc_channel) {
                case 3:
                    P1OUT &= ~BIT4 & ~BIT5 & ~BIT6;
                    break;
                case 2:
                    P1OUT = (P1OUT & (~BIT4 & ~BIT6)) | (BIT5);
                    break;
                case 1:
                    P1OUT = (P1OUT & (~BIT4 & ~BIT5)) | (BIT6);
                    break;
                }
                //reset everything
                sampled = 0;
                //initiates the data transfer and conversion process for the SLAVE(AD7265).
                P1OUT &= ~BIT7;
        ////        Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
                for (counter_timer = 4; counter_timer > 0; counter_timer--)
                {
                    UCB0IE |= UCTXIE;                     // Enable TX interrupt
                    __bis_SR_register(LPM0_bits | GIE);   // enable global interrupts, enter LPM0
                    __no_operation();                     // For debug,Remain in LPM0
                    sampled = (sampled << 8) + (SPIRXData & 0xFF);
                }
                adc_b[buffer_count-1][adc_channel-1] = ((sampled >> 2) & 0xFFF) << 4; // 12bit ADC
                adc_b[buffer_count-1][adc_channel-1] = adc_b[buffer_count-1][adc_channel-1] >> 4;

                P1OUT |= BIT7;
            }
	    }
	    low_pass(adc_b);
	    // UART take 238us, (1/460800)*10*11
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'U');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 'W');
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (frame_num & 0xFF));
        for (adc_channel = 3; adc_channel > 0; adc_channel--)
        {
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (filter[(adc_channel-1)] & 0xF00)>>8);
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (filter[(adc_channel-1)] & 0xFF)>>0);
        }
        frame_num++;
        __no_operation();                     // For debug,Remain in LPM0

	}

	return 0;
}

// TIMER_A ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TA0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    counter_timer += counter_timer;
    while (!(UCB0IFG&UCTXIFG));
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, SPITXData);

}
// SPI_B ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB0IV,USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;                // Vector 0 - no interrupt
        case USCI_SPI_UCRXIFG:
              SPIRXData = UCB0RXBUF;
//              UCB0IFG &= ~UCRXIFG;
              __bic_SR_register_on_exit(LPM0_bits);// Wake up to setup next TX
              break;
        case USCI_SPI_UCTXIFG:
              UCB0TXBUF = SPITXData;             // Transmit characters
              UCB0IE &= ~UCTXIE;
              break;
        default: break;
    }
}

//UART_A ISR
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector=USCI_A0_VECTOR
//__interrupt void USCI_A0_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
//  {
//    case USCI_NONE: break;
//    case USCI_UART_UCRXIFG:
//      while(!(UCA0IFG&UCTXIFG));
//      UCA0TXBUF = UCA0RXBUF;
//      __no_operation();
//      break;
//    case USCI_UART_UCTXIFG: break;
//    case USCI_UART_UCSTTIFG: break;
//    case USCI_UART_UCTXCPTIFG: break;
//    default: break;
//  }
//}

