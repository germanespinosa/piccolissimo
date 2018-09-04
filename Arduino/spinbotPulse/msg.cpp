#include "msg.h"
#define DEBUG_LED_ON   PORTD |= (1<<DEBUG_LED)
#define DEBUG_LED_OFF    PORTD  &= ~(1<<DEBUG_LED)


typedef void (*msgUserFunc)(char *);
volatile static msgUserFunc msg_user_function;
char msg_state      = MSG_ST_RESET;
char msg_bit_index  = 0;
char msg_byte       = 0;
char msg_byte_index = 0;
char msg_bytes[MSG_LEN+1];
char cc=0;

ISR(ANALOG_COMP_vect) // RAISING EDGE
{
  switch (msg_state)
  {
    case MSG_ST_RESET:
    {
      msg_state=MSG_ST_WAIT;
      MSG_RESET_TIMER(MSG_TIMER/2);
      msg_byte=0;
      msg_bit_index=0;
      break;
    }      
    case MSG_ST_WAIT:
    {
      msg_state=MSG_ST_START;
      break;
    }      
    case MSG_ST_INMSG:
    {
      msg_byte+=(1 << (msg_bit_index+4));
      break;
    }
  }
}

void msg_configure(void (*tempUserFunction)(char *))
{
  SREG &= ~(1<<SREG_I);

  ACSR &= ~(1<<ACIE); //disable interrupts on AC
  ACSR &= ~(1<<ACD); //switch on the AC
  ACSR &= ~(1<<ACBG); //set pin AIN0
  AC_REGISTER &= ~(1<<ACME); //set pin AIN1

  ACSR |= ((1<<ACIS1) | (1<<ACIS0));
  
  ACSR |= (1<<ACIE);
  SREG |= (1<<SREG_I);

  msg_user_function=tempUserFunction;
  msg_set_timer(MSG_TIMER);
}


void msg_set_timer(int interval)
{
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  MSG_RESET_TIMER(interval);
  TCCR1B |= (1 << WGM12);   // CTC mode
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts  
}

char msg_checksum(char *msg_bytes)
{
  char cs=0;
  for (int i=0;i<MSG_LEN;i++)
    cs=cs^msg_bytes[i];
  return ~cs & 240;
}

void msg_byte_handler(char b)
{
  msg_bytes[msg_byte_index++]=b;
  if (msg_byte_index>MSG_LEN)// if message over 
  {
    MSG_STOP_TIMER;
    msg_state=MSG_ST_RESET;
    char checksum=msg_checksum(msg_bytes);
    msg_bytes[MSG_LEN+1]=checksum;
    if (checksum==msg_bytes[MSG_LEN])
    {
      noInterrupts();           // disable all interrupts
      msg_user_function (msg_bytes);
      interrupts();           // disable all interrupts
    }
    msg_byte_index=0;
  }
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  switch (msg_state)
  {
    case MSG_ST_WAIT://timeout for check bit
    {
      msg_state=MSG_ST_RESET;
      MSG_STOP_TIMER;
      break;
    }
    case MSG_ST_START://checkbit received
    {
      msg_state=MSG_ST_INMSG;
      MSG_RESET_TIMER(MSG_TIMER);
      break;
    }
    case MSG_ST_INMSG:
    {
      msg_bit_index++;
      if (msg_bit_index==4)// full byte received
      {
        msg_byte_handler(msg_byte);
        msg_byte=0;
        msg_bit_index=0;
      }
    }
  }
}

