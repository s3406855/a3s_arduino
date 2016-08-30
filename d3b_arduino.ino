/*
 * TO-DO LIST
 * ~~~~~~~~~~
 * 
 * -- Check Atmega32/ousb schematic and double check pin choices. (Note: some hacking of the ousb board will be required to get all the analog inputs needed)
 * -- Make sure the logic makes sense, and is what the hardware boys need
 * -- Double check the PID constructor to make sure it's correctly set up (look in the .cpp file that came in the library)
 * 
 */

 /*
  * PINOUT FOR ARDUINO
  * 
  * DIGITAL I/O             ANALOG I
  * 0   N/A                 A0    N/A
  * 1   N/A                 A1    ACCELEROMETER
  * 2   LCD                 A2    K_P
  * 3   LCD                 A3    K_I
  * 4   LCD                 A4    K_D
  * 5   LCD                 A5    REFERENCE / SETPOINT
  * 6   N/A
  * 7   STEPPER DIRECTION
  * 8   STEPPER PWM
  * 9   N/A
  * 10  N/A
  * 11  LCD
  * 12  LCD
  * 13  N/A
  */

#define OSCILLATE // if this line is uncommented, the controller will bypass the PID loop and oscillate slowly from max left to max right
#define USE_LCD // comment this out if LDC use is NOT needed to save program space / increase efficiency

#include <PID_v1.h>

#ifdef USE_LCD
#include <LiquidCrystal.h>
#endif

/*
 * "CLICKS" - it was mentioned at one point that the shock adjustment might move in clicks of around 15 degrees each.
 * I don't think the mountain bike shock does this but i'm not sure.
 * If it turns out that the adjustment moves continuously, all we need to do is set MAX_CLICKS to 400 (720 deg / 1.8 deg)
 * and set steps_per_click to 1.
 */
#define MAX_CLICKS 48 // number of clicks in either direction to get 720 degrees

// holds the current rotation of the motor (one unit = one 'click' on shock)
volatile int8_t position_count = 0;

// define a couple of pins for ease of reference - these should be fine for Arduino but need to be verified for OUSB
volatile uint8_t direction_pin = 7; // On ousb use PB0, pin 1
volatile uint8_t step_pin = 8; // On ousb use PB1, pin 2

// Accelerometer input pins
//volatile int8_t accel_top = A0; // On ousb use PA0, pin 40 - TOP ACCELEROMETER NOT NEEDED - VALUE HARD CODED
volatile uint8_t accel_bot = A1; // On ousb use PA1, pin 39

// Gain set pins
volatile uint8_t k_p_pin = A2; // On ousb use PA2, pin 38
volatile uint8_t k_i_pin = A3; // On ousb use PA3, pin 37
volatile uint8_t k_d_pin = A4; // On ousb use PA4, pin 36

// Reference point set pin
volatile uint8_t ref_pin = A5; // On ousb use PA5, pin 35

// Doubles required for PID library. K values and REF read in during setup() from trimpots
volatile double K_P, K_I, K_D, REF, PID_IN, PID_OUT;

// threshold can be adjusted so we're not reacting to every minor bump (PID will probably never be exactly zero)
volatile int8_t threshold = 0.5;

// one 'click' on the shock = multiple motor steps - number TBC, assuming approx 8 for now
volatile uint8_t steps_per_click = 8;

/*
 * Min and Max delay time sets the min and max stepping speed of the motor
 * Used to set a delay time between steps in response to a PID calculation
 * Clock period at 12MHz is 83.33 nanoseconds
 */
volatile uint16_t min_delay = 1000; // microseconds
volatile uint16_t max_delay = 100000;

// Delay time to control the PWM signal
volatile uint32_t delay_us;

// Creat a PID object
PID a3s(&PID_IN, &PID_OUT, &REF, K_P, K_I, K_D, DIRECT);

// Accelerometer resting value
int16_t acc_offset;

/*
 * LCD for testing / config
 * Initialize the library with the numbers of the interface pins
 * NEED TO REASSIGN SOME PINS BECAUSE THERE IS A CLASH WITH PREVIOUSLY DEFINED OUTPUTS
 */
#ifdef USE_LCD
LiquidCrystal lcd( 12, 11, 5, 4, 3, 2 );
#endif

void setup(void)
{
  // DDRs
  pinMode(direction_pin, OUTPUT); // set direction pin as output (pin number specified in globals)
  pinMode(step_pin, OUTPUT);  // set stepper signal pin as output (pin number specified in globals)
  
  pinMode(A0, INPUT); // Analog pin to INPUT
  pinMode(A1, INPUT); // Analog pin to INPUT
  pinMode(A2, INPUT); // Analog pin to INPUT
  pinMode(A3, INPUT); // Analog pin to INPUT
  pinMode(A4, INPUT); // Analog pin to INPUT
  pinMode(A5, INPUT); // Analog pin to INPUT
  
  // Set ADC reference voltage
  analogReference(DEFAULT); // sets upper limit to 5v

  // Read gain and ref values
  K_P = analogRead(k_p_pin);
  K_I = analogRead(k_i_pin);
  K_D = analogRead(k_d_pin);
  REF = analogRead(ref_pin);

  //turn the PID on
  a3s.SetMode(AUTOMATIC);

  // Calibrate the accelerometer - find out where it's sitting and determine an offset
  // Idea - to take 5 or so readings over 5 seconds and take an average, then this result should be the offset
  // NEW THOUGHT - DO WE EVEN NEED TO DO THIS, OR CAN WE JUST SET THE REFERENCE TO BE EQUAL TO THE RESTING VALUE?
  for (unsigned short i = 0; i < 5; i++)
  {
    acc_offset += analogRead(accel_bot);
  }
  acc_offset = acc_offset / 5;

  #ifdef USE_LCD
  lcd.begin(16, 2); // 16 columns, 2 rows
  #endif
}

void loop(void)
{
  // if OSC definition at top is not commented out, the following test code will execute forever
  #ifdef OSCILLATE
    delay_us = 20000; // 20 milliseconds
    while (1)
    {
      goToMax();
      goToMin();
    }
  #endif

  /*
   * Ignore LCD unless it is "turned on"
   */
  #ifdef USE_LCD
  String p, i, d, r;
  
  p = (K_P / 1024);
  i = (K_I / 1024);
  d = (K_D / 1024);
  r = (REF / 1024);

  lcd.setCursor(0, 0);
  lcd.print("P=");
  lcd.setCursor(2, 0);
  lcd.print(p);

  lcd.setCursor(8, 0);
  lcd.print("I=");
  lcd.setCursor(10, 0);
  lcd.print(i);

  lcd.setCursor(0, 1);
  lcd.print("D=");
  lcd.setCursor(2, 1);
  lcd.print(d);

  lcd.setCursor(8, 1);
  lcd.print("R=");
  lcd.setCursor(10, 1);
  lcd.print(r);
  #endif
  
  // Read accelerometers
  PID_IN = analogRead(accel_bot);
  
  // Do a PID calculation
  a3s.Compute();
  
  delay_us = get_delay(&PID_OUT);
  
  if (PID_OUT > threshold) // pid commands a positive adjustment
  {
    if (position_count < MAX_CLICKS) // still below clockwise max?
    {
      StepForward();
      /*
       * Every time this function is called, a 'click' will have been achieved on the shock. If more than one click is required, the final delay() in the
       * function will carry over to the next run of this whole routine. There will be an additional delay due to the clock cycles required to get back to
       * the function. It's probably not a big deal, but we should minimize unnecessary code prior to the loop - ie. keep as much housekeeping as possible
       * in the setup() routine, and avoid declaring variables in loop().
       */
    }
  }
  else if (PID_OUT < (-1 * threshold)) // pid commands a negative adjustment
  {
    if (position_count > -MAX_CLICKS) // still above ccw max?
    {
      StepBack();
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

void goToMax(void)
{
  // 720 degrees clockwise
  do
  {
    StepForward();
  } while (position_count < MAX_CLICKS);
}

void goToMin(void)
{
  // 720 degrees counterclockwise
  do
  {
    StepBack();
  } while(position_count > (-MAX_CLICKS));
}

void returnToZero(void)
{
  if (position_count < 0)
  {
    // cw steps to get back to zero
    do
    {
      StepForward();
    } while (position_count < 0);
  }
  else if (position_count > 0)
  {
    // ccw steps to get back to zero
    do
    {
      StepBack();
    } while (position_count > 0);
  }
}

void StepForward(void)
{
  // set direction bit to HIGH
  digitalWrite(direction_pin, HIGH);
  
  // now loop as many times as required to make one full 'click' on the shock
  for (unsigned short i = 0; i < steps_per_click; i++)
  {
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(delay_us);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(delay_us);
  }
  position_count++;
}

void StepBack(void)
{
  // set direction bit LOW
  digitalWrite(direction_pin, LOW);

  for (unsigned short i = 0; i < steps_per_click; i++)
  {
    digitalWrite(step_pin, HIGH);
    delay(delay_us);
    digitalWrite(step_pin, LOW);
    delay(delay_us);
  }
  position_count--;
}

