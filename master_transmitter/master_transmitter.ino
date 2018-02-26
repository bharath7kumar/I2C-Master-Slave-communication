#include <avr/io.h>
#define F_CPU 16000000UL  // 16 MHz
#include <util/delay.h>

#include <avr/interrupt.h>
#include <util/twi.h>

#define SLAVE_ADDRESS 0x01

volatile int adc_value; // contains the received value

uint8_t ongoing_transmission = 0;

// interrupt routine for the timer0 overflow interrupt
ISR(TWI_vect)
{ // react on TWI status and handle different cases
  uint8_t stat = TWSR & 0xF8; // mask out the prescaler bits
  switch(stat)
  {
    
    case 0x20:
    case 0x30:
    case TW_START:  // start transmitted
         ongoing_transmission = 1;
         TWDR = (SLAVE_ADDRESS << 1) | 0x00;
         TWCR &= ~((1<<TWSTA)); // clear TWSTA
         _delay_ms(20);
         
    break;
  
    case 0x18: 
    case 0x28: // SLA+R transmitted, ACK received 
         ADCSRA |= (1<<ADSC);
         while(!(ADCSRA & (1<<ADIF)));
         adc_value=ADC;
         TWDR = adc_value;
         TWCR &= ~((1<<TWSTA));
         _delay_ms(20);
    break;
  }
  TWCR |=   (1<<TWINT);  // hand over to TWI hardware
}

int main(void) 
{
  //initialise ADC
   ADMUX = 1<<REFS0 ;
   ADCSRA = 1<<ADEN | 1<<ADPS1 | 1<<ADPS2 ;
  
  // TWI setup
  sei(); // enable global interrupt
  // TWI-ENable , TWI Interrupt Enable
  TWBR = 72;
  TWCR |= (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
  
  while(1)
  {
      // initiate new transmission if 
      //    no transmission is in progress
      if (ongoing_transmission==0)
      {
          TWCR |= (1<<TWSTA);
          _delay_ms(20);
      }  
  }
 
}



