
#include <Arduino.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#define BIT(a) (1 << (a))

static float z = 2 * 1.0e-3; // duration that pulse is on HIGH
float pulse_per_z; 
static float t0 = 0.0, ep, ei;
static float t, dt, tp = 0;
static float t2, dt2, tp2 = 0, t1, dt1, tp1; // to test pulse timing

// adc inputs (PID arguments)
float y1 = -1, y2 = -1, y3 = -1;


void launch_controller(float y1, float y2, float y3);
void ADC_setup();

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

  // set timer compare registers
  // OCR1A = 16e6 / 64.0 * 0.02 = 5000
  OCR1A = 60535 + pulse_per_z; // pulse width for pin 7 to w2_counts
  OCR1B = 60535 + pulse_per_z; // adc conversion
    
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

  t0 = micros() * 1.0e-6; // read initial time

  sei(); // enable interrupts

  while(1) launch_controller(y1,y2,y3);

}



ISR(TIMER1_OVF_vect)
{
  // turn pin 7 on
  PORTD |= BIT(7);

/*
  // test PORTD state
  Serial.print("\nPORTD = ");
  Serial.print(PORTD);

  t1 = micros() * 1.0e-6 - t0;
*/  

  // set pulse time to 5000 counts (0.02 s)
  TCNT1 = 60535;

}

ISR(TIMER1_COMPA_vect)
{
  // turn pin 7 off
  PORTD &= ~BIT(7);

 // set timer to continue after w2_counts until overflow
  TCNT1 = 60535 + pulse_per_z;

}

ISR(TIMER1_COMPB_vect)
{

  volatile unsigned int x = 0;
  int i = 0; // counter
  float t, sum = 0;
  int n = 200;
  float n_inv = 1.0 / n;

  static const float ADC_to_V = 1/1023.0*5;
  static const int ADC_CHANNELS = 3; // 3 channels
  
  // ADC values get stored in the ADC_read array
  volatile float ADC_read[ADC_CHANNELS] = {0, 0, 0};


  // ADC conversion loop
  while(i<3)
  {
    for (int k = 0; k < n; k++)
    {
    ADCSRA |= BIT(ADSC); // start conversion
    while(ADCSRA & BIT(ADSC)) x++; // poll
    sum += ADC; // measure and add to sum
    }

  // convert to voltage
  ADC_read[i] = (float)sum*n_inv*ADC_to_V; 
  i++;

  // change ADC channel (A1 -> A3 -> A5)
  ADMUX += 2; 

  // reset sum for next iteration
  sum = 0; 
  
  }

  // reset ADC channel    
  ADMUX = 0;
  ADMUX |= BIT(MUX0); // reads analog pin1
  ADMUX |= BIT(REFS0);

  y1 = ADC_read[0]; // drive motor
  y2 = ADC_read[1]; // right front wheel
  y3 = ADC_read[2]; // left front wheel

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

  void launch_controller(float y1, float y2, float y3)
{
  float kp = 50, ki = 10, kd = 1.001;
  
  const float target_slip = -0.2;
  float u1, y2_inv = 1/(y2+0.00001), CV;

  // timing
  float t = micros()*1e-6;
  float dt = t - tp;
  tp = t;

  // proportional error
  float actual_slip = ((y1 - y2) * y2_inv);
  float e = target_slip - actual_slip;

  // error derivative
  float ed = (e - ep)/dt;
  ep = e;

  // error integral
  ei += e*dt;
  
  u1 = kp * e + ki * ei + kd * ed;

  // saturation
  //if (u1 < 0.0) u1 = 0.0;
  //if (u1 > 5.0) u1 = 5.0;

  // convert desired voltage to required pulse width:
  // duty cycle = voltage / maximum voltage (5V) = pulse width / 0.02 ms
  // => pulse width = (voltage * 0.02) / 5V = voltage * 0.04
  z = u1 * 0.04;
  pulse_per_z = 16e6/64.0 * z;

  // changes time left to overflow based on desired voltage
  // calculated by PID controller
  // the higher the desired voltage is, the more the program stays in the 
  // overflow interrupt which keeps pin 7 on, increasing duty cycle accordingly.
  OCR1A = 60535 + pulse_per_z;

  //Serial.print("\nslip = ");
  Serial.print(actual_slip,8);Serial.print("\n");
}

void ADC_setup()
  {
  
  cli(); // disable interrupts
  
  ADMUX = 0;
  ADMUX |= BIT(MUX0); // read analog pin1 (A1)
  ADMUX |= BIT(REFS0); // set Vcc as reference

  ADCSRA = 0; // reset
  ADCSRA |= BIT(ADEN); // enable ADC
  ADCSRA |= BIT(ADPS2) | BIT(ADPS1); // ADC prescaler = 64
  
  sei(); // enable interrupts
  
  }
  
void loop()
{
// not used
}
