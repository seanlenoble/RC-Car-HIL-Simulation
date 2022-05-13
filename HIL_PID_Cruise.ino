
#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#define BIT(a) (1 << (a))

static float z = 2*1.0e-3; // duration that pulse is on HIGH
float pulse_per_z;
static float t0 = 0.0;
static float t2, dt2, tp2 = 0;
static float t, dt, tp = 0;
static float ep, ei; 

// adc inputs (PID arguments)
float y1 = -1, y2 = -1, y3 = -1;

// sets pulse width
float servo_command( float z);

void cruise_controller(float y1, float y2, float y3);
void ADC_setup();

// interrupts: OVF and COMPA - PWM
//             COMPB - servo_command
//             ADC conversions and channel assignment


void setup()
{

  Serial.begin(2000000);

  // counts needed for obtaining desired pulse width
  pulse_per_z = 16e6/64.0 * z;
  
  cli();  // disable interrupts

  // set bit 7 to output
  DDRD |= BIT(7); 

  // clear timer1 registers
  TCCR1A = 0;
  TCCR1B = 0;

  // set timer compare register
    OCR1B = 65535 + pulse_per_z;

  // set timer1 interrupt mask / register
  TIMSK1 = BIT(OCIE1A) | BIT(OCIE1B) | BIT(TOIE1);

  // clear previous compare match and ovf interrupts
  TIFR1 |= BIT(OCF1A) | BIT(OCF1B) | BIT(TOV1);
  
  // set timer1 prescaler to 64 and turn on timer
  TCCR1B |= BIT(CS11) | BIT(CS10);

  // turn on CTC mode (clear timer on compare match)
  TCCR1B |= BIT(WGM12);


  // set timer initial value to overflow immediately
  // and turn on pin 7
  TCNT1 = 65535;
 
  ADC_setup();

  t0 = micros() * 1.0e-6; // read initial time

  sei(); // enable interrupts

  while(1) cruise_controller(y1,y2,y3);


   
}


void ADC_setup()
  {
   cli(); // disable interrupts
  
  ADMUX = 0;
  ADMUX |= BIT(MUX0); // read analog pin1 (A1)
  ADMUX |= BIT(REFS0); // set Vcc as reference

  ADCSRA = 0; // reset
  ADCSRA |= BIT(ADEN); // enable ADC
  ADCSRA |= BIT(ADIE); // enable ADC interrupt
  ADCSRA |= BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2); // ADC prescaler = 128
  
  sei(); // enable interrupts
  
  }

ISR(TIMER1_OVF_vect)
{

  // turn pin 7 on
  PORTD |= BIT(7);
  
/*
  // test PORTD state
  //Serial.print("\nPORTD = ");
  //Serial.print(PORTD);


  tp2 = t2;
  t2 = micros() * 1.0e-6;
  dt2 = t2 - tp2;

  // test pulse width timing
  Serial.print("\ndt = ");
  Serial.print(dt2, 5); 
*/

  // set pulse time to 5000 counts ( 0.02 s)
  TCNT1 = 60535;
}


ISR(TIMER1_COMPA_vect)
{
  // turn pin 7 off
  PORTD &= ~BIT(7);

  //Serial.print("\nPORTD = ");
  //Serial.println(PORTD);
  
 // set timer to continue until overflow
 // pulse stays low until 0.02 ms is complete
  TCNT1 = 60535 + pulse_per_z;
  
}

ISR(TIMER1_COMPB_vect)  // set new pulse width
{
  pulse_per_z = servo_command(z);

}

  // the channel ID makes sure that every time the ADC interrupt is called
  // it takes in data for the right channel and then moves on to the next
  // it resets at the end of the if statement
  static int channel_id = 1;

  
  static int i = 0, k = 0;
  static float sum = 0;
  const int n = 200;
  const float n_inv = 1.0 / n;

  static const float ADC_to_V = 1/1023.0*5;
  static const int ADC_CHANNELS = 3; // 3 channels
  
  // ADC values get stored in the ADC_read array
  volatile float ADC_read[ADC_CHANNELS] = {0, 0, 0};


ISR(ADC_vect)
{
  
  // measure and add to sum
  sum += ADC; 

  // count to 200
  k++;

  // take average and set to right channel
  if (k > n) 
    {
      if ( ADMUX == 65 && channel_id == 1) // channel 1
      {
      ADC_read[0] = sum*n_inv*ADC_to_V; // convert to voltage

      ADMUX = 67;
      }

      if ( ADMUX == 67 && channel_id == 3) // channel 3
      {
      ADC_read[1] = sum*n_inv*ADC_to_V;

      ADMUX = 69;
      }

      if ( ADMUX == 69  && channel_id == 5) //channel 5
      {
        
      ADC_read[2] = sum*n_inv*ADC_to_V;

      ADMUX = 65;
      }

      //  reset
      k = 0;
      sum = 0;

      // designate next channel
      channel_id+=2;
      if (channel_id > 5) channel_id = 1;
    }
    
  y1 = ADC_read[0];
  y2 = ADC_read[1];
  y3 = ADC_read[2];

/* 
  Serial.print("\n"); 
  Serial.print(y1);
  Serial.print(",");
 
  Serial.print(y2);
  Serial.print(",");

  Serial.print(y3);
  Serial.print(",");  
  Serial.print("\n"); 
*/
}

void cruise_controller(float y1, float y2, float y3)
{
 /*unsigned int init = 0;

  if (init == 0)
  {
    ADCSRA |= BIT(ADSC);
    init = 1;
  }*/
  
float kp = 10, ki = 10, kd = 1.0001;
 
  float u1, CV, current_speed, target_speed = 15.0;

  //since 5V (max) gives 26.3 m/s, convert y2 voltage to wf by 1V = 5.26 m/s
  current_speed = ( y2 * 5.26 );

   // timing
  float t = micros()*1e-6;
  float dt = t - tp;
  tp = t;
  
  float e = target_speed - current_speed;
  float ed = (e - ep) / dt;
  ep = e;
  ei += e * dt;


  u1 = kp * e + ki + ei + kd * ed;
   
  // saturation 
 // if (u1 > 5) u1 = 5.0;
  //if (u1 < 0) u1 = 0.0;
  
  if (current_speed < 0.001) current_speed = 0;
  if (current_speed > 26.3) current_speed = 26.3;

  // convert desired voltage to required pulse width:
  // duty cycle = voltage / maximum voltage (5V) = pulse width / 0.02 ms
  // => pulse width = (voltage * 0.02) / 5V = voltage * 0.04
  z = u1 * 0.0125;


 // Serial.print("\nspeed = ");
 // Serial.print(current_speed,8);Serial.print("\n");

}

float servo_command(float z)
// set pulse width 
{
  if ( z < 0 ) z = 0;
  if ( z > 2500*1.0e-6) z = 2500*1.0e-6;
  
  pulse_per_z = 16e6/64.0 * z;

  // set new pulse width
   OCR1A = 60535 + pulse_per_z;

}


void loop()
{
// not used
}
