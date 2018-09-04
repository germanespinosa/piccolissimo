#include "timers.h"
#include "Arduino.h"
#define AC_REGISTER ADCSRB

#define MSG_TIMER 4
#ifndef MSG_TIMER_SCALE
#define MSG_TIMER_SCALE ((1 << CS11)|(1 << CS10))
#endif
#define MSG_STOP_TIMER TCNT1  = 0; TCCR1B &= ~MSG_TIMER_SCALE
#define MSG_RESET_TIMER(interval) MSG_STOP_TIMER;  OCR1A = interval; TCCR1B |= MSG_TIMER_SCALE
#ifndef MSG_LEN
#define MSG_LEN         3
#endif

#define MSG_START       0

#define MSG_ST_RESET    0
#define MSG_ST_WAIT     1
#define MSG_ST_START    2
#define MSG_ST_INMSG    3
#define MSG_ST_DONE     4

#ifndef MSG_LED
#define MSG_LED 4
#define MSG_LED_ON   PORTD |= (1<<MSG_LED)
#define MSG_LED_OFF  PORTD  &= ~(1<<MSG_LED)
#endif


void msg_configure(void (*tempUserFunction)(char *));
void msg_set_timer(int interval);
void msg_reset_timer(int interval);
void msg_stop_timer();
char msg_checksum(char *msg_bytes);
void msg_byte_handler(char b);
void msg_timer_handler();
void msg_signal_raising_edge();
void msg_signal_falling_edge();
void msg_start_analog_compare();

