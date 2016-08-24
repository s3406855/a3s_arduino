#include <PID_v1.h>

// holds the current rotation of the motor (one unit = one 'click' on shock)
volatile int8_t position_count = 0;

// define a couple of pins for ease of reference - these may not be the best choice - pulled them out of a hat
volatile int8_t direction_pin = 1; // PB0
volatile int8_t step_pin = 2; // PB1

// Accelerometer input pins
volatile int8_t accel_top = 40; // PA0
volatile int8_t accel_bot = 39; // PA1

// Gain set pins
volatile int8_t k_p_pin = 38; // PA2
volatile int8_t k_i_pin = 37; // PA3
volatile int8_t k_d_pin = 36; // PA4

// Reference point set pin
volatile int8_t ref_pin = 35; // PA5

// Doubles required for PID library. K values and REF read in during setup() from trimpots
volatile double K_P, K_I, K_D, REF, PID_IN, PID_OUT;

// threshold can be adjusted so we're not reacting to every minor bump (PID will probably never be exactly zero)
volatile int8_t threshold = 0.5;

// one 'click' on the shock = multiple motor steps - number TBC, assuming approx 8 for now
volatile int8_t steps_per_click = 8;

/*
 * Min and Max delay time sets the min and max stepping speed of the motor
 * I think the arduino library specifies milliseconds but this may be too course for us
 * 1 clock cycle at 12MHz is 83.33 nanoseconds so it should be possible to drill down to a finer delay time
 */
volatile int16_t min_delay = 1; // ms? micros or nanos would be better
volatile int16_t max_delay = 100;

// Delay time to control the PWM signal
volatile int16_t delay_ms;

// Creat a PID object
PID a3s(&PID_IN, &PID_OUT, &REF, K_P, K_I, K_D, DIRECT);

void setup(void)
{
  // set direction pin as output (pin number specified in globals)
  pinMode(direction_pin, OUTPUT);
  
  // set stepper signal pin as output (pin number specified in globals)
  pinMode(step_pin, OUTPUT);

  // Set up ADC
  analogReference(DEFAULT); // sets upper limit to 5v

  // Read gain and ref values
  K_P = analogRead(k_p_pin);
  K_I = analogRead(k_i_pin);
  K_D = analogRead(k_d_pin);
  REF = analogRead(ref_pin);

  //turn the PID on
  a3s.SetMode(AUTOMATIC);
}

void loop(void)
{
  // Read accelerometers
  PID_IN = analogRead(accel_top) - analogRead(accel_bot);
  
  // Do a PID calculation
  a3s.Compute();
  
  delay_ms = get_delay(&PID_OUT);
  
  if (PID_OUT > threshold) // pid commands a positive adjustment
  {
    if (position_count < 48) // 48 clicks in each direction assuming 15 deg per click
    {
      // set direction bit to HIGH
      digitalWrite(direction_pin, HIGH);

      // now loop as many times as required to make one full 'click' on the shock
      for (unsigned short i = 0; i < steps_per_click; i++)
      {
        digitalWrite(step_pin, HIGH);
        delay(delay_ms);
        digitalWrite(step_pin, LOW);
        delay(delay_ms);
      }
      /*
       * Every time this loop finishes, a 'click' will have been achieved on the shock. If more than one click is required, the final delay() in the above
       * loop will carry over to the next run of this whole routine. There will be an additional delay due to the clock cycles required to restart the loop.
       * It's probably not a big deal, but we should minimize unnecessary code prior to the loop - ie. keep as much housekeeping as possible in the setup()
       * routine, and avoid declaring variables in loop().
       */
      position_count++;
    }
  }
  else if (PID_OUT < (-1 * threshold)) // pid commands a negative adjustment
  {
    if (position_count > -48)
    {
      // set direction bit LOW
      digitalWrite(direction_pin, LOW);

      for (unsigned short i = 0; i < steps_per_click; i++)
      {
        digitalWrite(step_pin, HIGH);
        delay(delay_ms);
        digitalWrite(step_pin, LOW);
        delay(delay_ms);
      }
      position_count--;
    }
  }

  /*
   * At this point, one 'click' on the shock should have occured. The program continues
   * and if the PID calculations still command adjustment, then another 'click' is performed,
   * otherwise nothing happens until the next time PID returns an adjustment above the 
   * threshold.
   */
}

/*
 * Delay is calculated by taking the value returned by the PID calculation and 
 * equating it to a delay time. The higher the value returned by PID, the smaller 
 * the delay time.
 */
int16_t get_delay(double *pid)
{
  // since a large pid value maps to a small delay time, first subtract pid from the max possible value
  int16_t temp = 65536 - *pid;
  
  float frac = (float)temp / (float)65536;
  
  return ( (frac * max_delay) + min_delay );
}

