

#include<avr/io.h>
#include<avr/interrupt.h>
#define ONE 1

int Ignition =0 , sw_camera=0;
int val1,val2,value;
int read;


void init(); // function for initialization of ports 
void Ignition_sw(); // function for interrupt of ignition switch
int ADC_beam(); // function for Analog signal read
int ADC_visor(); // function for ADC initialization
void  camera_sw(); // function for interrupt of camera switch
void power_window();
void wiper_pwm();


int main()
{
  Serial.begin(9600); ///main function 
  TCNT0=0; // COUNT is initialized
  sei(); // GLOBAL interrupt is initialized
  init();
  Ignition_sw();
  camera_sw();
  
  
  sei();
  init();
  TCNT2=0x00;
  TCCR2A|=0x00;

  
  while(1)
{
  
  if(Ignition==1)
    {
     while(sw_camera==1) // Ignition is on
     { 
     int a;       
     a=ADC_visor();
     PORTD |= (ONE << PD4);
     Serial.println(val1); //in adc    
     if(val1>400)
       {
         _delay_ms(1000);
         PORTD|=(1<<PD3);
         PORTD&=~(1<<PD5);
       }
    if(val1<=400)
       {
         _delay_ms(1000);
         PORTD|=(1<<PD5);
         PORTD&=~(1<<PD3);
       }
     }

    {
      PORTD&= ~(ONE << PD4);
      PORTD&=~(1<<PD5);
      PORTD&=~(1<<PD3);
    }
  
   
      ADC_beam();
      Serial.println(val2);
      PORTD |= (1<<PD7);//tail light
      if(val2<=250)                //checking the adcvlue
        
        { 
          pwm_100();//indicates it is night so it generates pwm signals with 100% duty cycle
        }
      else if(val2>250 && val2<=450)            
        { 
          pwm_75();
        }
      else if(val2>450 && val2<=600)            
        { 
          pwm_50();
        }
      else  
        { 
          pwm_00();
        }
    
    
    PORTD|=(1<<PD5);
                  value=ADC_wiper();
      Serial.println(value);
      
      if(value<100)
      {
        OCR2B=50;
        wiper_pwm();
      }
      else if(value>100 && value<=500)
      {
        OCR2B=128;
        wiper_pwm();

      }
      else if(value>500 && value<=900)
      {
        OCR2B=192;
        wiper_pwm();
      }
      else if(value>900)
      {
        OCR2B=230;
        wiper_pwm();

      }
      
   
   ADC_power();
    
    if(!(PINB&(1<<PB1)))
    {
      if(PINC&(1<<PC1)) // To rotate motor in anticlockwise
      {
      PORTB|=(1<<PB3);
     
      Serial.println(PINC &(1<<PC1));

      PORTB &=~(ONE<<PB2);
      PORTC &=~(ONE<<PC3);
      }
      else
      {
      PORTC |=(ONE<<PC3);
      PORTB &=~(ONE<<PB2);
      }

    }
   
     if(!(PINC&(1<<PC5))) // To rotate motor in anticlockwise
      {
       PORTC &=~(ONE<<PC3);
       PORTB |=(ONE<<PB2);
      
      }
    
  }

    else
    {
     PORTD&= ~(ONE << PD4);
     PORTD&=~(1<<PD5);
     PORTD&=~(1<<PD3);
     PORTD &= ~(1 << PD7);            //making ports as 0
     PORTD &= ~(1 << PD6);            //making ports as 0
     TIMSK0 =0; 
      
     //next 2
      
    PORTB &=~(ONE<<PB2);
    PORTC &=~(ONE<<PC3);
    PORTC &=~(ONE<<PB0);
    PORTC &=~(ONE<<PB1);
    PORTD&=~(1<<PD6);
    PORTD&=~(1<<PD5);
    TIMSK2=0;
    }
  }
}

void Ignition_sw() // External Intrrupt of Ignition
{
EICRA |= (ONE << ISC00);
EIMSK |= (ONE << INT0);
}

ISR(INT0_vect) //ISR for igniton
{
  if(Ignition==1)
  {
    Ignition=0; // Ignition Switch is toggled
  }
  else
  {
    Ignition=1;
  }
}

void init() //Initialization function 
{
  DDRC &=~(ONE << PC0); // input for photoresistor
  PORTC &=~ (ONE << PC0);
  DDRC &=~(ONE << PC1);
  PORTC&=~(1<<PC1);
  DDRC &=~(ONE << PC2);
  PORTC &=~ (ONE << PC2);
  DDRC |= (ONE<<PC3);// motor recover
  PORTC &=~ (ONE << PC3);
  DDRC &=~(ONE << PC4);
  PORTC&=~(1<<PC4);
  DDRC &=~(1 << PC5);
  PORTC&=~(1<<PC5);

  
  DDRD&=~(1<<PD2);
  DDRD&=~(1<<PD1);
  PORTD|=(1<<PD2);
  PORTD|=(1<<PD1);
  DDRD |= (ONE<<PD3);// motor recover
  PORTD &=~ (ONE << PD3);
  DDRD |= (ONE << PD4);
  PORTD &=~ (ONE << PD4);  
  DDRD |= (ONE<<PD5);// motor action
  PORTD &=~ (ONE << PD5);
 DDRD|=(1<<PD6);
  PORTD&=~(1<<PD6);
  DDRD |=(1<<PD7);
  PORTD &=~(1<<PD7);
  
  DDRB |= (ONE<<PB3);// motor recover
  PORTB &=~ (ONE << PB3);

  DDRB &= ~(1<<PB1);    //Switch1
  PORTB|=(1<<PB1);


  DDRB &= ~(1<<PB2);    //Switch1
  PORTB|=(1<<PB2);

  DDRB &= ~(1<<PB4);    //Switch1
  PORTB|=(ONE << PB4);

}

int ADC_visor()
{
  ADMUX &=0X00;
  ADMUX |=(1<<REFS0);
  ADCSRA |=(1<<ADEN);
  ADCSRA |=(1<<ADSC);
  while(ADCSRA & (1<<ADSC));//wait upto conversation complete
  val1=ADC; //is a macro is has dig output
  return 0;
}

int ADC_beam()
{
  ADMUX &=0X00;
  ADMUX |=(1<<MUX1);
  ADMUX |=(1<<REFS0);
  ADCSRA |=(1<<ADEN);
  ADCSRA |=(1<<ADSC);
  ADC=0;
  while(ADCSRA & (1<<ADSC));//wait upto conversation complete
  val2=ADC; //is a macro is has dig output  
  return 0;
}

int ADC_wiper(void)
{ 
  int value1;
  ADMUX &=0X00;
  ADMUX |=(1<<MUX2);
  ADMUX |=(1<<REFS0);
  ADCSRA |=(1<<ADEN);
  ADCSRA |=(1<<ADSC);
  ADC=0;
  while(ADCSRA & (1<<ADSC));//wait upto conversation complete
  value1=ADC; //is a macro is has dig output  
  return value1;
}

int ADC_power(void)
{
  ADMUX &=0X00;
  ADMUX |=(1<<MUX0);
  ADMUX |=(1<<REFS0);
  ADCSRA |=(1<<ADEN);
  ADCSRA |=(1<<ADSC);
  ADC=0;
  while(ADCSRA & (1<<ADSC));//wait upto conversation complete
  read=ADC; //is a macro is has dig output  
  return 0;
}

void timer_enable()
  {
   TCNT0 =0;
   TCCR0A |=(1<<WGM01);
   TCCR0A &=~(1<<WGM00);
   TCCR0B &=~(1<<WGM02);
   }

void pwm_100()
{
  OCR0A=255;
  OCR0B=10;
  TCCR0B |=((1<<CS02)|(1<<CS00));
  TCCR0B &=~(1<<CS01);
TIMSK0 |=((1<<OCIE0A)|(1<<OCIE0B));
}

void pwm_75()
{
  OCR0A=255;
  OCR0B=70;
  TCCR0B |=((1<<CS02)|(1<<CS00));
  TCCR0B &=~(1<<CS01);
TIMSK0 |=((1<<OCIE0A)|(1<<OCIE0B));
}
    
void pwm_50()
{
  OCR0A=255;
  OCR0B=128;
  TCCR0B |=((1<<CS02)|(1<<CS00));
  TCCR0B &=~(1<<CS01);
  TIMSK0 |=((1<<OCIE0A)|(1<<OCIE0B));
}

void pwm_00()
{
  TCCR0B &=~((1<<CS02)|(1<<CS00));
  PORTD &=~(1<<PD6);
  //TCCR0B &=~(1<<CS01);
  TIMSK0 =0x00;
}
  
void wiper_pwm()
{
  TCCR2B|=((1<<CS20)|(1<<CS22)|(1<<CS21));
  TIMSK2|=((1<<OCIE2A)|(1<<OCIE2B));
  OCR2A=255;
}



void camera_sw() //Pin Change Interrupt for ignition
{
  PCICR |= (ONE << PCIE0);
  PCMSK0 |= (ONE << PCINT4);
}

ISR(TIMER2_COMPA_vect)
{
  PORTD|=(1<<PD7);
}
ISR(TIMER2_COMPB_vect)
{
  PORTD&=~(1<<PD7);
}


ISR(TIMER0_COMPA_vect)
{
  PORTD &= ~(HIGH<<PD6);  
}
ISR(TIMER0_COMPB_vect)
{
  PORTD |= (HIGH<<PD6);
}

ISR(PCINT0_vect)
{
  
   sw_camera=sw_camera^1; //camera switch interrupt
}
