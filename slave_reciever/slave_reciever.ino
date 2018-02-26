#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL  // 16 MHz
#include <util/delay.h>
#include <util/twi.h>

#define SLAVE_ADDRESS 0x01

uint8_t adc_value; // the value to send

uint8_t ongoing_transmission = 0;

int busy();
void sendnibble(char nibble);
void lcdcmd(char cmd);
void lcddata(char dat);
void lcdstring(char *str);
void lcdnum(int num);


// interrupt routine for TWI message handling
ISR(TWI_vect)
{   
  // react on TWI status and handle different cases
  uint8_t stat = TWSR & 0xF8; // mask out the prescaler bits
  switch(stat)
  {
    
    case TW_SR_SLA_ACK:   // own SLA+R received, acknoledge sent
         ongoing_transmission = 1;
         TWCR &= ~((1<<TWSTO));
         _delay_ms(20);
    break;

    case TW_SR_DATA_ACK: // data received, ACK returned
                   
         adc_value = TWDR;
         lcdcmd(0x01);
         lcdnum(TWDR);
         TWCR |= (1<<TWSTO);  // write stop bit
         TWCR &= ~(1<<TWSTA); // clear start bit
         ongoing_transmission = 0;
         _delay_ms(40);
    break;
   }
    TWCR |= (1<<TWINT);  // set TWINT  activate TWI hardware
}

int main(void)
{
  DDRB = 0x0F;           //input for last four pins, rest output
  DDRD = 0b11101111;     //all pins output
  PORTB = 0b00110000;    //setting 2 pins for buttons 
  PORTD = 0b00010000;

    PORTD |= 1<<PD5 | 1<<PD6 | 1<<PD7;
    _delay_us(15);

    lcdcmd(0x02);
    lcdcmd(0x28);        // LCD
    lcdcmd(0x0C);        // INITIALISE
    lcdcmd(0x01);
    lcdcmd(0x80);

  // TWI setup
  sei(); // enable global interrupt
  TWBR = 72;
  // set slave address to 0x01, ignore general call
  TWAR = (SLAVE_ADDRESS << 1) | 0x00;
  // TWI-ENable , TWI Interrupt Enable
  TWCR |= (1<<TWEA) | (1<<TWEN) | (1<<TWIE); 

  // infinite loop
  for (;;)
  {
    if(ongoing_transmission==0)
    {
        TWCR |= (1<<TWEA);
        _delay_ms(20);  
    }
    _delay_ms(20);
  }
}

//send commands 
void lcdcmd(char cmd)
{
    while(busy());                     //wait while busy
    sendnibble((cmd >> 0x04) & 0x0F);  //send second nibble
    PORTD &= ~(1<<PD5);                //clear RS 
    PORTD &= ~(1<<PD6);                //clear RW
    PORTD |= (1<<PD7);                 //set   E  
    _delay_us(1000);
    PORTD &= ~(1<<PD7);                //clear R 

    _delay_us(10000);

    sendnibble(cmd & 0x0F);      //send first nibble
    PORTD &= ~(1<<PD5);          //clear RS      
    PORTD &= ~(1<<PD6);          //clear RW
    PORTD |= (1<<PD7);           //set E
    _delay_us(1000);
    PORTD &= ~(1<<PD7);          //clear E

    _delay_us(10000);
}

//send data
void lcddata(char dat)
{
    while(busy());                    //wait while busy
    sendnibble((dat >> 0x04) & 0x0F); //send second nibble
    PORTD |= 1<<PD5 ;                 //set RS
    PORTD &= ~(1<<PD6);               //clear RW  
    PORTD |= 1<<PD7 ;                 //set E
    _delay_us(1000);
    PORTD &= ~(1<<PD7);               //clear E

    _delay_us(10000);

    sendnibble(dat & 0x0F);           //send first nibble 
    PORTD |= 1<<PD5 ;                 //set RS
    PORTD &= ~(1<<PD6);               //clear RW
    PORTD |= 1<<PD7 ;                 //set E
    _delay_us(1000);
    PORTD &= ~(1<<PD7);               //clear E 

    _delay_us(10000);
}

//send number
void lcdnum(int num)
{
   char string[2];
   itoa(num,string,10);
   lcdstring(string); 
}

//send string
void lcdstring(char *str)
{
    while(*str > 0)
    {
        lcddata(*str++);    
    }
}
//check busy flag
int busy()
{
    PORTD |= 1<<PD6;            // mask RS
    PORTD &= ~(1<<PD5);         // clear RW
    PORTD |= 1<<PD7;            // mask E
    if(PORTB == PORTB|(1<<PB4)) // check for most significant pin 
        return 0;
    else
        return 1;  
}

//send nibble one by one since we are using 4-bit transfer
void sendnibble(char nibble)
{
    PORTB &= ~(1<<PB0 | 1<<PB1 | 1<<PB2 | 1<<PB3); //clear all data pins
    _delay_us(10);
    PORTB |= (((nibble >> 0x00) & 0x01) << 0);
    PORTB |= (((nibble >> 0x01) & 0x01) << 1);
    PORTB |= (((nibble >> 0x02) & 0x01) << 2);     //send data through pins
    PORTB |= (((nibble >> 0x03) & 0x01) << 3);     //by shifting them
    _delay_us(10);
}
