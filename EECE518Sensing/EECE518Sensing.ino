#include <CapacitiveSensor.h>

/*
 * Pin configurations
 */
CapacitiveSensor cs_2_5 = CapacitiveSensor(2,5); // Pin 2 is send pin, Pin 5 is receive (sensor) pin
CapacitiveSensor cs_2_4 = CapacitiveSensor(2,4); // Pin 2 is send pin, Pin 4 is receive (sensor) pin
CapacitiveSensor cs_2_3 = CapacitiveSensor(2,3); // Pin 2 is send pin, Pin 3 is receive (sensor) pin
const int linearPot1      = A3;
const int linearPot2      = A2;
const int fslpSenseLine   = A1;
const int fslpDriveLine1  = 8;
const int fslpDriveLine2  = A0;
const int fslpBotR0       = 9;

int cap1out = 0;
int cap2out = 0;
int cap3out = 0;

/*
 * Setup function. It runs once whenever the reset button is pressed
 * or the board is powered up.
 */
void setup() {
  // Set up the cap sensors
  cs_2_5.set_CS_AutocaL_Millis(0xFFFFFFFF);
  cs_2_4.set_CS_AutocaL_Millis(0xFFFFFFFF);
  cs_2_3.set_CS_AutocaL_Millis(0xFFFFFFFF);

  // Setup softpot linear pots
  digitalWrite(linearPot1, HIGH); // enable pullup resistor
  digitalWrite(linearPot2, HIGH); // enable pullup resistor

  // Setup serial
  Serial.begin(9600);
}

/*
 * Main loop function
 */
void loop() {
  // Perform the raw cap readings
  int cap1 = (int) cs_2_3.capacitiveSensor(30);
  int cap2 = (int) cs_2_4.capacitiveSensor(30);
  int cap3 = (int) cs_2_5.capacitiveSensor(30);

  // Map the raw cap readings to something meaningful
  cap1 = mapCap(cap1);
  cap2 = mapCap(cap2);
  cap3 = mapCap(cap3);

  // Perform the raw pot readings
  int pot1 = analogRead(linearPot1); // softpot
  int pot2 = analogRead(linearPot2); // softpot
  int pot3 = (fslpGetPressure() == 0) ? 0 : fslpGetPosition(); // pololu

  // Map the raw pot readings to something meaningful
  pot1 = mapPot(pot1, true);
  pot2 = mapPot(pot2, true);
  pot3 = mapPot(pot3, false);

  if (cap1 == 1) {
    cap1out = 1;
    cap2out = 0;
    cap3out = 0;
  } else if (cap2 == 1) {
    cap1out = 0;
    cap2out = 1;
    cap3out = 0;
  } else if (cap3 == 1) {
    cap1out = 0;
    cap2out = 0;
    cap3out = 1;
  }

  // Prepare the serial output
  char report[25];
  sprintf(report, "%d %d %d %d %d %d\n", pot1, pot2, pot3, cap1out, cap2out, cap3out);
  Serial.print(report);
  
  delay(50);
}

/*
 * Maps readings from linear potentiometers.
 * 
 * @param pot The raw pot readings
 * @param softpot True if softpot pots, false if pololu.
 * @return +1 to zoom in, -1 to zoom out, 0 to do nothing.
 * 
 * Softpot mapping:
 * Raw readings are from 1024 to 0. When not pressed, the raw data is
 * from 1000 to 1024; so we'll map that to 0. So 500 to 1000, we'll
 * map it to zoom in. 0 to 500, we'll map it to zoom out.
 * 
 * Pololu mapping:
 * Raw readings are from 0 to 1024. When not pressed, the raw data is
 * from 0 to 20; so we'll map tthat to 0. So 20 to 500, we'll map it to
 * zoom out. 500 to 1024 we'll map to zoom in.
 */ 
int mapPot(int pot, bool softpot) {
  if (softpot) {
    if (pot > 1000) {
      return 0;
    } else if (pot > 500 && pot <= 1000) {
      return 1;
    } else {
      return -1;
    }
  } else {
    if (pot < 20) {
      return 0;
    } else if (pot >= 20 && pot < 500) {
      return -1;
    } else {
      return 1;
    }
  }
}

/*
 * Maps readings from capacitive readings.
 * 
 * @param cap The raw cap readings
 * @return +1 if capacitive readings detected a touch; 0 otherwise.
 * 
 * Mapping:
 * When untouched, the capacitive readings are < 5. Anything greater
 * is considered a touch.
 */
int mapCap(int cap) {
  if (cap > 5) {
    return 1;
  } else {
    return 0;
  }
}

/*
 * This function follows the steps described in the FSLP
 * integration guide to measure the position of a force on the
 * sensor. The return value of this function is proportional to
 * the physical distance from drive line 2, and it is between
 * 0 and 1023. This function does not give meaningful results
 * if fslpGetPressure is returning 0.
 * 
 * Example code in: https://github.com/pololu/fslp-led-strip-arduino-demo
 */
int fslpGetPosition()
{
  // Step 1 - Clear the charge on the sensor.
  pinMode(fslpSenseLine, OUTPUT);
  digitalWrite(fslpSenseLine, LOW);

  pinMode(fslpDriveLine1, OUTPUT);
  digitalWrite(fslpDriveLine1, LOW);

  pinMode(fslpDriveLine2, OUTPUT);
  digitalWrite(fslpDriveLine2, LOW);

  pinMode(fslpBotR0, OUTPUT);
  digitalWrite(fslpBotR0, LOW);

  // Step 2 - Set up appropriate drive line voltages.
  digitalWrite(fslpDriveLine1, HIGH);
  pinMode(fslpBotR0, INPUT);
  pinMode(fslpSenseLine, INPUT);

  // Step 3 - Wait for the voltage to stabilize.
  delayMicroseconds(10);

  // Step 4 - Take the measurement.
  analogReset();
  return analogRead(fslpSenseLine);
}

/*
 * This function follows the steps described in the FSLP
 * integration guide to measure the pressure on the sensor.
 * The value returned is usually between 0 (no pressure)
 * and 500 (very high pressure), but could be as high as
 * 32736.
 * 
 * Example code in: https://github.com/pololu/fslp-led-strip-arduino-demo
 */
int fslpGetPressure()
{
  // Step 1 - Set up the appropriate drive line voltages.
  pinMode(fslpDriveLine1, OUTPUT);
  digitalWrite(fslpDriveLine1, HIGH);

  pinMode(fslpBotR0, OUTPUT);
  digitalWrite(fslpBotR0, LOW);

  pinMode(fslpSenseLine, INPUT);

  pinMode(fslpDriveLine2, INPUT);

  // Step 2 - Wait for the voltage to stabilize.
  delayMicroseconds(10);

  // Step 3 - Take two measurements.
  analogReset();
  int v1 = analogRead(fslpDriveLine2);
  analogReset();
  int v2 = analogRead(fslpSenseLine);

  // Step 4 - Calculate the pressure.
  // Detailed information about this formula can be found in the
  // FSLP Integration Guide.
  if (v1 == v2)
  {
    // Avoid dividing by zero, and return maximum reading.
    return 32 * 1023;
  }
  return 32 * v2 / (v1 - v2);
}

/*
 * Example code in: https://github.com/pololu/fslp-led-strip-arduino-demo
 */
void analogReset()
{
#if defined(ADMUX)
#if defined(ADCSRB) && defined(MUX5)
    // Code for the ATmega2560 and ATmega32U4
    ADCSRB |= (1 << MUX5);
#endif
    ADMUX = 0x1F;

    // Start the conversion and wait for it to finish.
    ADCSRA |= (1 << ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
#endif
}
