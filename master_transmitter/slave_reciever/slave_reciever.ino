#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL  // 16 MHz
#include <util/delay.h>
#include <util/twi.h>

#define SLAVE_ADDRESS 0x01

uint8_t adc_value; // the value to send

uint8_t ongoing_transmission = 0;

// interrupt routine for TWI message handling
ISR(TWI_vect)
{ Serial.println(TWSR);
  
  // react on TWI status and handle different cases
  uint8_t stat = TWSR & 0xF8; // mask-out the prescaler bits
  switch(stat)
  {
    
    //TWCR|= 1<<TWEA ;
   
    
    case TW_SR_SLA_ACK:   // own SLA+R received, acknoledge sent
          //Serial.println("recieved");       //TWDR = adc_value;
         ongoing_transmission = 1;
         TWCR &= ~((1<<TWSTO));
    break;

    case TW_SR_DATA_ACK: // data received, ACK returned
                   
         adc_value = TWDR;
         Serial.println(TWDR);
         TWCR |= (1<<TWSTO);  // write stop bit
         TWCR &= ~(1<<TWSTA); // clear start bit
         ongoing_transmission = 0;
         //TWCR |= (1<<TWEA);
         //Serial.println(ongoing_transmission);
         //TWCR &= (1<<TWSTA);
    break;
    
    /*case TW_SR_LAST_DATA: // last byte transmitted ACK received     
         TWCR |= (1<<TWEA); // set TWEA to enter slave mode
         ongoing_transmission = 0;
    break;*/
   }
    TWCR |= (1<<TWINT);  // set TWINT -> activate TWI hardware
}

int main(void)
{
  // TWI setup
  sei(); // enable global interrupt
  TWBR = 72;
  // set slave address to 0x01, ignore general call
  TWAR = (SLAVE_ADDRESS << 1) | 0x00;
  // TWI-ENable , TWI Interrupt Enable
  TWCR |= (1<<TWEA) | (1<<TWEN) | (1<<TWIE); 


  //initialise ADC
   /*ADMUX = 1<<REFS0 ;
   ADCSRA = 1<<ADEN | 1<<ADPS1 | 1<<ADPS2 ;
   */  
 Serial.begin(9600);
  // infinite loop
  for (;;)
  {
    //TWCR |= 1<<TWIE; 
    //_delay_ms(20);
    if(ongoing_transmission==0)
    {
        TWCR |= (1<<TWEA);
        _delay_ms(20);  
    }
    _delay_ms(20);
    /*ADMUX &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2));
    ADCSRA |= (1<<ADSC);
    while(!(ADCSRA & (1<<ADIF)));
    adc_value=ADC;
    Serial.println(TWSR);*/    
    Serial.println(TWSR);
  }
}
