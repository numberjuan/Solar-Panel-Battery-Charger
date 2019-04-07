/* Pin 1.0 A0
 * pin 1.1 A1
 * Pin 1.2 A2
 * Pin 1.3 A3
 * Pin 1.4 SS
 * Pin 1.5 SPI Clock
 * Pin 1.6 SPI SOMI
 * Pin 1.7 SPI SIMO
 * Pin 2.0
 * Pin 2.1
 * Pin 2.2
 * Pin 2.3
 * Pin 2.4
 *
*/

//Need to do: Put in final manipulation values for ADC,
#include <msp430g2553.h>
#include <intrinsics.h>
//#include <math.h.>

volatile char DC = 0; //Vale to store RX into so read is finished
volatile char ReadyToSend[4]; //Data ready to send to master when requested
volatile int JustRecieved[4];
volatile char SendCounter = 1; //Counter to know which byte to send
volatile char Channel = 4; //Variable to see which channel we are reading
volatile char Pin = 0;

void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	P1OUT = 0;
	P1SEL = BIT4 | BIT5 | BIT6 | BIT7; //Set up SPI functionality for pins
	P1SEL2 = BIT4 | BIT5 | BIT6 | BIT7;
	P2DIR = 0xFF; // All port 2 output 0 to save power
	P2OUT = 0;
	
    // Calibrate clock
    DCOCTL = 0;
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // Setup timer A0
    TACCR0 = 5000; //20ms timer
    TACTL = TASSEL_2 | ID_2 | MC_1; // SMCLK/4, up mode
    TACCTL0 = CCIE; // enable timer 0 interrupt

    // Setup ADC10
    ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10IE; // 3.3V and GND references, 16 sample cycles, ADC off, interrupts enabled
    ADC10CTL1 = SHS_0 | ADC10DIV_0 | ADC10SSEL_0 | CONSEQ_0; // activate on SC bit, Straight binary, no divisions,single channel single conversion
    ADC10AE0 = BIT0 | BIT1 | BIT2 | BIT3; //Set channel A0, A1, A2, and A3 as analog inputs

    //SPI Setup
    UCB0CTL1 |= UCSSEL_2 | UCSWRST; // SMCLK, put in reset
    UCB0CTL0 |= UCCKPH | UCMSB | UCMODE_1 | UCSYNC; //4 wire mode (So have slave select), MSB first, synchronous, slave mode
    UCB0BR1 = 0;
    UCB0BR0 = 10; // SMCLK/10 = 100khz for SPI clock
    //UCA0MCTL = 0; //No modulation
    UCB0CTL1 &= ~UCSWRST; // Take out of reset

    IE2 |= UCB0RXIE; //Enable SPI receive interrupt
    __bis_SR_register(GIE); // enable interrupts

    //char i = 8;
    //for(i=4; i>0; i--){
      //  ReadyToSend[i-1] = 78+i;
   // }
   // UCB0TXBUF = ReadyToSend[0]; //Load first value into buffer so we can send m
    for(;;){}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void badprog_timer_a0(void) {
  TACCTL0 = 0; //Disable further interrupts
  if(Channel == 0){ Channel = 4;}
  Channel = Channel - 1; // Decrement the channel
  ADC10CTL0 &= ~ENC;  // Stop conversion so the control register can be changed (pg 555 of MSP430 family guide)
  ADC10CTL1 = 0; //Reset register so new channel can be selected
  switch(Channel){
  case 0 : { ADC10CTL1 |= INCH_0; break;} //Set which channel to read on
  case 1:  { ADC10CTL1 |= INCH_1; break;}
  case 2 : { ADC10CTL1 |= INCH_2; break;}
  case 3 : { ADC10CTL1 |= INCH_3; break;}
  }
   ADC10CTL0 |= ADC10ON; //Turn on ADC
   ADC10CTL0 |= (ENC | ADC10SC); // Start conversion

  }

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void) {
    while(ADC10CTL1 & BUSY);
    ADC10CTL0 &= ~ENC;  // Stop conversion so the control register can be changed (pg 553 of MSP430 family guide)
    ADC10CTL0 &= ~ADC10ON; // Turn off ADC
   /* switch(Channel){
    case 0 : { VoltageCH0 = ADC10MEM; break;}
    case 1 : { VoltageCH3 = ADC10MEM; break;}
    case 2 : { VoltageCH4 = ADC10MEM; break;}
    case 3 : { VoltageCH5 = ADC10MEM; UCA0TXBUF = VoltageCH0; break;}
    }*/ //Old code
    switch(Channel){
    case 0 : { ReadyToSend[0] = ADC10MEM >> 2; JustRecieved[0] = ADC10MEM; UCB0TXBUF = ReadyToSend[0]; break;} //Store data, manipulate data and store in ready to send array
    case 1 : { ReadyToSend[1] = ADC10MEM >> 2;  JustRecieved[1] = ADC10MEM; break;}//Are not final Manipulation values!!!!
    case 2 : { ReadyToSend[2] = ADC10MEM >> 2;  JustRecieved[2] = ADC10MEM; break;}
    case 3 : { ReadyToSend[3] = ADC10MEM >> 2;  JustRecieved[3] = ADC10MEM; break;}
    }
    ADC10CTL0 &= ~ADC10IFG; // clear interrupt flag
    TACCTL0 = CCIE; //Enable Timer 0 interrupt

}

#pragma vector = USCIAB0RX_VECTOR // SPI RX interrupt
__interrupt void USCIAB0RX_ISR(void){
    Pin = P1IN;
    Pin &= BIT4;
    DC = UCB0RXBUF;
    //if(UCB0RXBUF == 0xAA){
    if(Pin == BIT4){

        TACCTL0 = 0; //Disable timer interrupt until we are done sending data
        UCB0TXBUF = ReadyToSend[SendCounter];
        SendCounter = SendCounter + 1;
        if(SendCounter == 4){SendCounter = 0;}
        else if(SendCounter == 1){ TACCTL0 = CCIE;}
        //if(SendCounter == 4){SendCounter = 0;}
                //If done sending data, reset counter and start up timer again
    }
   // }
    IFG2 &= ~UCB0RXIFG; // Clear RX flag
}

