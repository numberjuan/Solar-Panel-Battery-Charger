/* Pin 1.0 SS1 (Active high)
 * pin 1.1 UART RX
 * Pin 1.2 UART TX
 * Pin 1.3 SS2 (Active high)
 * Pin 1.4
 * Pin 1.5 SPI Clock
 * Pin 1.6 SPI SOMI
 * Pin 1.7 SPI SIMO
 * Pin 2.0 Relay1
 * Pin 2.1 Relay2
 * Pin 2.2 Relay3
 * Pin 2.3 Relay4
 * Pin 2.4
 *
*/

//Need to do list: Put in final values for PC commands, put in which slaves hold which byte numbers, logic for UART RX
#include <msp430g2553.h>
#include <intrinsics.h>

//volatile char TXMode = 0; //Sets if using UART (0) or SPI (1)
volatile char ReceiveCounter = 0; //Counts bytes received
volatile char DATA[8]; //byte data for 7 ADC readings (transformed by slaves)
volatile char RFlags = 0; //Stores the RX flags for UART and SPI
void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	//Port 1 Setup
	P1DIR = BIT0 | BIT3; //SET SS1 and SS2 as output
    P1OUT = 0; //Both Slave Selects inactive
	P1SEL = BIT1 | BIT2 | BIT5 | BIT6 | BIT7; //Set up UART and SPI functionality
	P1SEL2 = BIT1 | BIT2 | BIT5 | BIT6 | BIT7;

	//Port 2 Setup
	P2DIR = BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7; //Set all unused pins as output
	P2OUT = 0;

	// Calibrate clock
	DCOCTL = 0;
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;

	//UART Setup
    UCA0CTL0 = 0;
    UCA0BR1 = 0;
    UCA0BR0 = 0x0068; // 1MHZ, 9600 Baud Rate
    UCA0MCTL = UCBRS_0;
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
    IE2 |= UCA0RXIE; //Enable RX interrupt

    //SPI Setup
    UCB0CTL1 |= UCSSEL_2 | UCSWRST; // SMCLK, put in reset
    UCB0CTL0 |= UCCKPH | UCMSB | UCMST | UCMODE_0 | UCSYNC; //3 wire mode, MSB first, synchronous
    UCB0BR1 = 0;
    UCB0BR0 = 10; // SMCLK/10 = 100khz for SPI clock
    //UCA0MCTL = 0; //No modulation
    UCB0CTL1 &= ~UCSWRST; // Take out of reset

    IE2 |= UCA0RXIE | UCB0RXIE;   // Enable UART and SPI Receive interrupts
    _BIS_SR(GIE);             // Enable the global interrupt

   // P1OUT = BIT0; UCB0TXBUF = 0xAA;

    for(;;){} //infinite for loop
}

#pragma vector = USCIAB0TX_VECTOR // SPI and UART TX interrupts
__interrupt void USCIAB0TX_ISR(void){
     switch(ReceiveCounter){ //Keeps track of which byte we are sending
        case 0: { UCA0TXBUF = DATA[0]; ReceiveCounter+=1; break;} //Turn on slave, store received value, send random stuff, increment counter
        case 1: { UCA0TXBUF = DATA[1]; ReceiveCounter+=1; break;}
        case 2: { UCA0TXBUF = DATA[2]; ReceiveCounter+=1; break;}
        case 3: { UCA0TXBUF = DATA[3]; ReceiveCounter+=1; break;}
        case 4: { UCA0TXBUF = DATA[4]; ReceiveCounter+=1; break;}
        case 5: { UCA0TXBUF = DATA[5]; ReceiveCounter+=1; break;}
        case 6: { UCA0TXBUF = DATA[6]; ReceiveCounter+=1; break;}
        case 7: { UCA0TXBUF = DATA[7]; ReceiveCounter=0; IE2 &= ~UCA0TXIE; break;}
     }
}

#pragma vector = USCIAB0RX_VECTOR // SPI and UART RX interrupts
__interrupt void USCIAB0RX_ISR(void){
    P1OUT = 0; //Set slaves to not send
    RFlags = IFG2;
    RFlags &= (UCA0RXIFG | UCB0RXIFG);
    if(RFlags == UCA0RXIFG) //IF UART interrupt
    { //Not final values for commands!!!
        if(UCA0RXBUF == 0x30) //Update data state when ASCII 0 sent
        {P1OUT = BIT0;  __delay_cycles(150); UCB0TXBUF = 0xAA; IFG2 &= ~(UCA0RXIFG|UCB0RXIFG) ;} //Tell slave1 to start sending data
        else if(UCA0RXBUF == 0X31)//Turn off entire system IF ASCII 1 RECIEVED
        {P2OUT = BIT0 | BIT1 | BIT2 | BIT3;}
        else if(UCA0RXBUF == 0X32)//turn off specific thing, ASCII 2
        {P2OUT |= BIT0;}
        else if(UCA0RXBUF == 0X33) // ASCII 3
        {P2OUT |= BIT1;}
        else if(UCA0RXBUF == 0X34) //ASCII 4
        {P2OUT |= BIT2;}
        else if(UCA0RXBUF == 0X35) //ASCII 5
        {P2OUT |= BIT3;}
        else if(UCA0RXBUF == 0X36)//Turn on system, ASCII 6
        {P2OUT = 0;}
    }
    else if(RFlags == UCB0RXIFG) //If SPI interrupt
    {
        switch(ReceiveCounter){ //Need to decide which slave to talk to (what is here is not final!!)
        case 0: { P1OUT = BIT0;  __delay_cycles(150); DATA[0] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;} //Turn on slave, store received value, send random stuff, increment counter
        case 1: { P1OUT = BIT0;  __delay_cycles(150); DATA[1] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;}
        case 2: { P1OUT = BIT0;  __delay_cycles(150); DATA[2] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;}
        case 3: { P1OUT = BIT3;  __delay_cycles(150); DATA[3] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;}
        case 4: { P1OUT = BIT3;  __delay_cycles(150); DATA[4] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;} //Turn on slave2 istead of slave1
        case 5: { P1OUT = BIT3;  __delay_cycles(150); DATA[5] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;}
        case 6: { P1OUT = BIT3;  __delay_cycles(150); DATA[6] = UCB0RXBUF; UCB0TXBUF = 0xAA; ReceiveCounter+=1; break;}
        case 7: { P1OUT = 0;  __delay_cycles(150); DATA[7] = UCB0RXBUF; ReceiveCounter=0; IE2 |= UCA0TXIE; break;}//Turn on UART TX interrupt
        }
    }
    IFG2 &= ~(UCA0RXIFG|UCB0RXIFG) ; // Clear RX flags
}
