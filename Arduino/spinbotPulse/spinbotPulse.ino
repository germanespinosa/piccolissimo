#include <Wire.h>
#include "msg.h"

#define SIGNAL_PROCESSING
#define SIGNAL_DIFFERENCE (snap_shot>signal_value_tmp?snap_shot-signal_value_tmp:0)
#define SIGNAL_CAPTURE    snap_shot=micros(); signal_value_tmp=signal_value
#define SIGNAL_IS_RAISING !is_up && signal_value_tmp!=last_signal_value
#define SIGNAL_IS_FALLING is_up && SIGNAL_DIFFERENCE>1500
#define SIGNAL_TIME_OUT   SIGNAL_DIFFERENCE>200000
#define SIGNAL_UPDATE     last_signal_value=signal_value_tmp


#define IS_ON(VALUE,START,STOP) ((STOP < START) && ((VALUE <= STOP) || (START < VALUE))) || ((STOP > START) && ((START < VALUE) && (VALUE <= STOP))) 


#define CORRECT_OFFSET(VALUE) VALUE>1?VALUE-1:VALUE<0?VALUE+1:VALUE

#define WIFI_OFF


#define MOTOR_HI 255
#define MOTOR_LO 0
#define MOTOR_PWM (current_thrust/2)
#define BRAKE_STATE HIGH
#define MOTOR_COMP 1.0

#define TIME_OUT    1000
#define MIN_CYCLE   250
#define TIME_SCALE  8
#define INT_MAX     2147483647
#define PEAK_LED    4
#define CONTROL_LED 2
#define ADDR        18
#define THRUST_LED  3
#define SENSOR_PIN  A0
#define PWM_HI      255
#define PWM_LO      0
#define PWM_WIDTH   255
#define MIN_MESSAGE 2

#define MOTOR 5
#define BRAKE 1
#define MOTOR_HI 255

#define PEAK_LED_OFF      PORTD |= (1<<PEAK_LED)
#define PEAK_LED_ON       PORTD  &= ~(1<<PEAK_LED)

#define CONTROL_LED_OFF   PORTD |= (1<<CONTROL_LED)
#define CONTROL_LED_ON    PORTD  &= ~(1<<CONTROL_LED)

#define THRUST_LED_OFF    PORTD |= (1<<THRUST_LED)
#define THRUST_LED_ON     PORTD  &= ~(1<<THRUST_LED)

#define MOTOR_OFF         PORTD  &= ~(1<<MOTOR)
#define MOTOR_ON          PORTD |= (1<<MOTOR)


#define MIN_SPEED(VALUE) (VALUE>=32?VALUE:0)

#define MODE_PWM   0
#define MODE_PULSE 1
#define MODE_TIMEOUT 2

#define GRANULARITY 10000.0
#define STEP 100.0


void receiveEvent(int bytes);
void setup();

unsigned long fstart = 0;
unsigned long bstart = 0;
unsigned long fstop = 0;
unsigned long bstop = 0;
unsigned long freq_counter = 0;
unsigned long freq_cycle = 0;
unsigned long in_len = 0;
unsigned long in_counter = 0;
unsigned long flen = 0;
unsigned long blen = 0;

bool is_up = false;
float foffset_start = 0;
float foffset_stop = 0;
float boffset_start = 0;
float boffset_stop = 0;
unsigned char pwm_value = PWM_LO;

unsigned long  signal_value = 0;
unsigned long  last_signal_value = 0;
unsigned long  signal_value_tmp = 0;
unsigned long  last_signal_value_tmp = 0;

unsigned long  signal_value_dif = 0;
unsigned long  snap_shot = 0;

uint32_t last_message_time;

unsigned char last_width, last_offset, last_thrust, checksum, cce, current_width, current_offset, current_thrust, prop_pwm;


int flymode = MODE_PWM;

unsigned long last_cycle_time;

bool is_spinning = false;
float fifty;
int v = 0;
void loop()
{
  bool fup = false;
  bool bup = false;

  SIGNAL_CAPTURE;
  freq_counter++;
  freq_counter = snap_shot - last_cycle_time;
  if (SIGNAL_IS_FALLING && flymode!=MODE_TIMEOUT)
  {
// process the cycle
    freq_cycle = freq_counter; // this is the full cycle lenght
    last_cycle_time = snap_shot; // reset the counter
    freq_counter = 0;
    in_len = freq_cycle - in_counter; //this is the lenght between the raising and falling edge.
    float in_len_offset= float(in_len)/float(freq_cycle)/2;
//
/// process the messages

    float width = float(current_width) / 240;  // read one character from the I2C

    if (width < 0.08)
    {
      flymode = MODE_PWM;
    }
    else
    {
      flymode = MODE_PULSE;
      is_spinning = true;
    }

    
    //float neutral = float(MOTOR_PWM) / 255 / 2 + MOTOR_COMP;
    float fwidth = width/2;//neutral * (1 + width);
    float bwidth = fwidth;//neutral * (1 - width);

    float ppwm = 128;
    if (fwidth<.5) 
      ppwm=(float(MOTOR_PWM) * MOTOR_COMP - (255 * fwidth))/(1-width);

    if (ppwm>255)
      prop_pwm=255;
    else
      if (ppwm<0)
        prop_pwm=0;
      else
        prop_pwm=ppwm;

    float foffset = CORRECT_OFFSET(float(current_offset) / 255-in_len_offset); 
    foffset_start = CORRECT_OFFSET(foffset-fwidth/2);
    foffset_stop = CORRECT_OFFSET(foffset+fwidth/2);
    
    float boffset = foffset>=.5?foffset-.5:foffset+.5;
    boffset_start = CORRECT_OFFSET(boffset-bwidth/2);
    boffset_stop = CORRECT_OFFSET(boffset+bwidth/2);

// end process 
   

    fstart= freq_cycle * (GRANULARITY * foffset_start)/ GRANULARITY;
    fstop = freq_cycle * (GRANULARITY * foffset_stop) / GRANULARITY;

    bstart= freq_cycle * (GRANULARITY * boffset_start)/ GRANULARITY;
    bstop = freq_cycle * (GRANULARITY * boffset_stop) / GRANULARITY;

    
    is_up = false;
  }
  if (SIGNAL_IS_RAISING )// && (freq_counter+in_len>=MIN_CYCLE))
  {
    in_counter = freq_counter;
    is_up = true;
  }
  if ((SIGNAL_TIME_OUT) && is_spinning)
  {
    current_width = 0;
    current_offset = 0;
    current_thrust = 0;
    analogWrite(THRUST_LED,255);
    analogWrite(MOTOR,0);
    flymode=MODE_TIMEOUT;
  }
  
  switch (flymode)
  { case MODE_PWM:
    {
      CONTROL_LED_OFF;
      analogWrite(THRUST_LED,PWM_WIDTH-MOTOR_PWM);
      digitalWrite(BRAKE,LOW);
      analogWrite(MOTOR,MOTOR_PWM);
      break;
    }
    case MODE_PULSE:
    {
      fup = (IS_ON(freq_counter,fstart,fstop));
      bup = (IS_ON(freq_counter,bstart,bstop));

      if (fup )
      {
        CONTROL_LED_ON;
        analogWrite(THRUST_LED,255);
        analogWrite(MOTOR,255);
        digitalWrite(BRAKE,LOW);
      }
      else
      {
        CONTROL_LED_OFF;
      }
      if (bup )
      {
        analogWrite(THRUST_LED,255);
        analogWrite(MOTOR,0);
        digitalWrite(BRAKE,LOW);
      }
      if (!bup && !fup)
      {
        analogWrite(THRUST_LED,PWM_WIDTH-prop_pwm);
        analogWrite(MOTOR,prop_pwm);
        digitalWrite(BRAKE,LOW);
      }
      break;
    }
    case MODE_TIMEOUT:
    {
        PEAK_LED_ON;
        delay(500);
        PEAK_LED_OFF;        
        delay(500);
        break;
    }
  }
  SIGNAL_UPDATE;
}

int pass_counter=0;
void set_values(unsigned char *msg)
{
  if (last_width == msg[0] && last_offset == msg[1] && last_thrust == msg[2]) 
    pass_counter++;
    else
    pass_counter=0;
  if (pass_counter>=MIN_MESSAGE)
  {
    current_width = msg[0];
    current_offset = msg[1];
    current_thrust = msg[2];
    if (current_width==0 && current_offset==0 && current_thrust ==0)
    { 
      is_spinning=false;
      flymode=MODE_PWM;
    }
  }
  last_width = msg[0];
  last_offset = msg[1];
  last_thrust = msg[2];
  signal_value = snap_shot;
  checksum = msg[3];
  cce = msg[4];
}

#ifdef WIFI_ON
void requestEvent()
{
  Wire.write(last_width);
  Wire.write(last_offset);
  Wire.write(last_thrust);
  Wire.write(checksum);
  Wire.write(cce);
}
#endif

void setup()
{
  flymode=MODE_PWM;
  last_cycle_time = micros();
  pinMode(10, INPUT);
  pinMode(PEAK_LED, OUTPUT);
  pinMode(CONTROL_LED, OUTPUT);
  pinMode(THRUST_LED, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(MSG_LED, OUTPUT);
  msg_configure(set_values);
  setPwmFrequency(THRUST_LED, 8);
  pinMode(SENSOR_PIN, INPUT);
  digitalWrite(PEAK_LED, HIGH);
  digitalWrite(CONTROL_LED, HIGH);
  digitalWrite(THRUST_LED, HIGH);
  digitalWrite(MSG_LED, HIGH);
  analogWrite(MOTOR, MOTOR_LO);
  digitalWrite(BRAKE, LOW);
while(0){
  CONTROL_LED_ON;
  delay(500);
  CONTROL_LED_OFF;
  delay(500);
}
  CONTROL_LED_ON;
  delay(500);
  CONTROL_LED_OFF;
  delay(500);
  CONTROL_LED_ON;
  delay(500);
  CONTROL_LED_OFF;
  delay(500);

#ifdef WIFI_ON
  Wire.begin(ADDR);
  Wire.onRequest(requestEvent);
#endif
}


void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

