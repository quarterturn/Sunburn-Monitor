/*************************************************** 
  Sunburn Monitor
  Uses SI1132 sensor and Adafruit SI1145 library modified to look for i2c ID 0x032 vs 0x045.
  Designed for ATMEGA168 or ATMEGA328 MCU 8 mHz 3.3v.
  
  Calculates max sun exposure before sunburn based on skin type,
  sun uv intensity, and sunscreen SPF.
  
  Skin type          Max Time        Color
  I                  67 min/UVI      pale white
  II                 100 min/UVI     beige
  III                200 min/UVI     tan or very light brown
  IV                 300 min/UVI     light brown
  V                  400 min/UVI     olive or medium brown
  VI                 500 min/UVI     dark brown
  
  Pins used (Arduino):
  A5 SCL
  A4 SDA
  9 piezo
  10 piezo
  2, 3, 4 buttons
  5 power control FET for UV sensor
 ****************************************************/

// for i2c
#include <Wire.h>
// library for Si1145/Si1132
// from here git clone https://github.com/adafruit/Adafruit_SI1145_Library.git
#include "Adafruit_SI1145.h"
// EEPROM stuff
#include <EEPROM.h>
// Bounce button library
// from here git clone https://github.com/thomasfredericks/Bounce-Arduino-Wiring.git
#include <Bounce2.h>
// toneAC library - louder than regular tone library
// from here https://code.google.com/p/arduino-tone-ac/downloads/detail?name=toneAC_v1.2.zip&can=2&q=
#include <toneAC.h>
// sleep
#include <avr/sleep.h>
// power management
#include <avr/power.h>
// interrupts
#include <avr/interrupt.h>
// watchdog timer
#include <avr/wdt.h>

// defines
// used to test for first-time EEPROM configuration
#define EE_CHECKBYTE_1 125
#define EE_CHECKBYTE_2 126
// stores the skin type
#define EE_SKIN_TYPE 127

// state machine states
#define STATE_POWER_OFF B00000000
#define STATE_CONFIG B00000001
#define STATE_RUNNING B00000010
#define STATE_COMPLETED B00000100

// set timeout
// how long to wait in millis for the user to set things or go back to sleep
// 30 seconds
#define SET_TIMEOUT 30000
// alarm timeout
// how long to play the alarm tone
// 15 seconds
#define MAX_ALARM_TIME 15000
// time between alarm beeps
#define ALARM_PAUSE_TIME 500
// maxiumum running time
// 12 hours - 720 minutes
// shut down after this time to prevent draining the battery
#define MAX_RUNNING_TIME 720

// WDT timer periods required for 1 minute of sleeping
// 4 sec wdt X 15 = 60 seconds
#define WDT_SLEEP_CYCLES 15

// low battery warning voltage
// non-decimal due to how we use an int for the
// internal voltage measurement
#define MIN_BATT_VOLTS 270

// toneAC note constants
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

// button pins
#define ON_OFF_PIN 2
#define SKIN_PIN 3
#define SET_PIN 4

// UV sensor power pin
#define UV_SENSOR_PWR_PIN 5

// button debounce time
#define DEBOUNCE_MS 20
// button long click time
#define LONG_CLICK_MS 1000

// variables

// enter config mode on powerup as default
volatile byte state = STATE_CONFIG;
// track setting time
unsigned long setTimer;
// track alarm time
unsigned long alarmTimer;
// track button hold time
unsigned long buttonTime;
// track total running time in minutes
// we add a minute each time we take a sample from the UV sensor
uint16_t runningTimer = 0;


// skin type
uint8_t skinType;
// skin type lookup table for sunburn minutes/UVI
static uint16_t skinTypeMinutesPerUVI[6] = {0, 67, 100, 200, 400, 500};
     
// minutes until sunburn
float minutesToBurn;

// watchdog timer sleep count
volatile byte wdtSleepCount = 0;

// spf for sunscreen
// map spf set button presses to spf
static uint8_t pressesPerSPF[6] = {0, 1, 10, 15, 30, 45};

// vcc voltage
int vccVoltage;

// ADCSRA temp
static uint8_t adcSettings;

// create a 'uv' object based on Adafruit_SI1145
Adafruit_SI1145 uv = Adafruit_SI1145();

// set up buttons
Bounce onOffButton = Bounce();
Bounce skinButton = Bounce();
Bounce setButton = Bounce();

//---------------------------------------------------------------------------------------------//
// setup
//---------------------------------------------------------------------------------------------//
void setup()
{
  
  // disable the wdt
  wdt_disable();
  // set up the UV sensor power pin
  pinMode(UV_SENSOR_PWR_PIN, OUTPUT);
  // turn it on for testing
  digitalWrite(UV_SENSOR_PWR_PIN, LOW);
  
  // set up the buttons
  // on/off
  pinMode(ON_OFF_PIN, INPUT);
  digitalWrite(ON_OFF_PIN, HIGH);
  onOffButton.attach(ON_OFF_PIN);
  onOffButton.interval(DEBOUNCE_MS);
  // start/stop
  pinMode(SKIN_PIN, INPUT);
  digitalWrite(SKIN_PIN, HIGH);
  skinButton.attach(SKIN_PIN);
  skinButton.interval(DEBOUNCE_MS);
  // start/stop
  pinMode(SET_PIN, INPUT);
  digitalWrite(SET_PIN, HIGH);
  setButton.attach(SET_PIN);
  setButton.interval(DEBOUNCE_MS);
  
  // read the EEPROM checkbytes
  if ((EEPROM.read(EE_CHECKBYTE_1) == 0xBA) && (EEPROM.read(EE_CHECKBYTE_2) == 0xDA))
  {
    // read the skin type from EEPROM
    skinType = EEPROM.read(EE_SKIN_TYPE);
  }
  else
  {
    // set the checkbytes
    EEPROM.write(EE_CHECKBYTE_1, 0xBA);
    EEPROM.write(EE_CHECKBYTE_2, 0xDA);
    // the skin type will default to 1
    // erring on the side of caution in case the user forgets to change it
    EEPROM.write(EE_SKIN_TYPE, 1);
  }
  
  // Serial.begin(9600);
  
  // Serial.println("Looking for SI1135 on i2c bus");
  delay(200);
  
  if (! uv.begin())
  {
    // Serial.println("Didn't find Si1135");
    // Serial.println("halting");
    toneAC(NOTE_C5, 10, 500, 0);
    while (1);
  }
  
  // take a UV measurement
  doMeasure();
  // take a voltage measurement
  vccVoltage = getBandgap();
  // Serial.print("VCC: ");
  // Serial.println(vccVoltage);

  // Serial.println("OK!");
  
  // play startup so the user knows the battery is good
  playStartup();
  
  // Serial.println("Going to sleep from power-on");
  
  // turn off the UV sensor
  digitalWrite(UV_SENSOR_PWR_PIN, HIGH);
  
  // go to state power off
  state = STATE_POWER_OFF;
  
}

//---------------------------------------------------------------------------------------------//
// main
//---------------------------------------------------------------------------------------------//
void loop()
{
  switch (state)
  {
    // config state
    case STATE_CONFIG:      
      // do config stuff
      // Serial.println("Entering config mode");
      doConfig();
      // if state is power off skip the rest of this case section
      if (state == STATE_POWER_OFF)
      {
        break;
      }
      else
      {
        // enable the WDT
        enable_watchdog(); 
        // go to sleep
        // Serial.println("Going to sleep after config mode");
        // delay(100);
        sleepNow();
      }
      break;
      
    // running state
    case STATE_RUNNING:
      // check if 1 minute/15 WDT sleep cycles has elapsed
      if (wdtSleepCount == 15)
      {
        // reset the wdt sleep count
        wdtSleepCount = 0;
        // increment the running timer
        runningTimer++;
        // check if we have been running for too long
        // to avoid draining the battery
        if (runningTimer == MAX_RUNNING_TIME)
        {
          // disable the WDT
          disable_watchdog();
          // switch to power down state
          // Serial.println("Exceeded runtime limit! Sleeping");
          state = STATE_POWER_OFF;
        }
        // do measuring and testing
        else
        {
          // test for power button
          // we only accept a long press
          // to prevent accidentally shutting it off
          if (onOffButton.update())
          {
            if (onOffButton.read() == LOW)
            {
              // start timing the button press
              buttonTime = millis();
              // loop until the button is released
              while(1)
              {
                if (onOffButton.update())
                {
                  if (onOffButton.read() == HIGH)
                  {
                    // exit the button hold loop
                    break;
                  }
                }
              }
              // see how long the button was held down
              // if less than the long click time
              if (millis() - buttonTime < LONG_CLICK_MS)
              {
                // do nothing
                // go on to do the measurement
                // Serial.println("Power press - too short");
              }
              // otherwise it was a long press
              // move to the power off state
              else
              {
                // disable the WDT
                disable_watchdog();
                // switch to power down state
                state = STATE_POWER_OFF;
                // exit the case section
                break;
              }
            }
          }
          // take an internal voltage measurement
          vccVoltage = getBandgap();
          // Serial.print("VCC: ");
          // Serial.println(vccVoltage);
          
          // if the battery is low play a low warning beep
          // otherwise play a very short beep
          if (vccVoltage <= MIN_BATT_VOLTS)
          {
            toneAC(NOTE_C4, 10, 150, 1);
          }
          else
          {
            shortBeep();
          }
          // take a UV measurement
          doMeasure();
          // check if we are out of time
          if (minutesToBurn <= 0)
          {
            // disable the WDT
            disable_watchdog(); 
            // go to state completed
            // Serial.println("Time to burn reached");
            state = STATE_COMPLETED;
          }
          // otherwise reset the WDT sleep count
          // and go back to sleep
          else
          {
            // reset the WDT sleep count
            wdtSleepCount = 0;
            // Serial.print("Minutes to burn: ");
            // Serial.println(minutesToBurn);
            // go back to sleep
            // Serial.println("Sleeping after measurement");
            // delay(100);
            sleepNow();
          }
        }
      }
      // otherwise just check the for the power button
      else
      {
        // test for power button
        // we only accept a long press
        // to prevent accidentally shutting it off
        if (onOffButton.update())
        {
          if (onOffButton.read() == LOW)
          {
            // start timing the button press
            buttonTime = millis();
            // loop until the button is released
            while(1)
            {
              if (onOffButton.update())
              {
                if (onOffButton.read() == HIGH)
                {
                  // exit the button hold loop
                  break;
                }
              }
            }
            // see how long the button was held down
            // if less than the long click time
            if (millis() - buttonTime < LONG_CLICK_MS)
            {
              // do nothing
            }
            // otherwise it was a long press
            // move to the power off state
            else
            {
              // switch to power down state
              state = STATE_POWER_OFF;
              // exit the case section
              break;
            }
          }
        }
        // Serial.print("WDT cycle: ");
        // Serial.println(wdtSleepCount);
        // delay(100);
        // go back to sleep
        sleepNow();
        
      }
      break;
      
    // completed state
    case STATE_COMPLETED:
      // Serial.println("Sunburn limit reached");
      // do stuff here once sunburn limit is reached
      playAlarm();
      // go to power down state
      state = STATE_POWER_OFF;
      break;
      
    // power off state (deep sleep)
    case STATE_POWER_OFF:
      // turn off the uv sensor
      digitalWrite(UV_SENSOR_PWR_PIN, HIGH);
      // reset the timer
      runningTimer = 0;
      // small pause
      delay(100);
      // play the shutdown sound
      playShutdown();
      // disable the wdt
      disable_watchdog(); 
      // attach interrupt
      attachInterrupt(0, pin2_isr, LOW);
      // go to sleep here
      // Serial.println("Going to sleep from power button");
      // delay(100);
      sleepNow();
      // execute after waking up
      // give the sensor time to power up
      delay(200);
      // go back to config state
      state = STATE_CONFIG;    
      break;
      
    // default
    default:
      break;
  }
}

//---------------------------------------------------------------------------------------------//
// function playStartup()
//
// Plays a rising sequence of notes to signal startup/coming out of deep sleep
//---------------------------------------------------------------------------------------------//
void playStartup()
{
  toneAC(NOTE_C4, 10, 150, 0);
  toneAC(NOTE_G4, 10, 150, 0);
  toneAC(NOTE_C5, 10, 150, 0);
  toneAC(NOTE_G5, 10, 150, 0);
  toneAC(NOTE_C6, 10, 150, 0);
  toneAC(0);
}

//---------------------------------------------------------------------------------------------//
// function playShutdown()
//
// Plays a falling sequence of notes to signal shutdown/entering deep sleep
//---------------------------------------------------------------------------------------------//
void playShutdown()
{
  toneAC(NOTE_C6, 10, 150, 0);
  toneAC(NOTE_G5, 10, 150, 0);
  toneAC(NOTE_C5, 10, 150, 0);
  toneAC(NOTE_G4, 10, 150, 0);
  toneAC(NOTE_C4, 10, 150, 0);
  toneAC(0);
}

//---------------------------------------------------------------------------------------------//
// function playStart()
//
// Plays a musical sequence to indicate measurement has begun
//---------------------------------------------------------------------------------------------//
void playStart()
{
  toneAC(NOTE_D5, 10, 150, 0);
  toneAC(NOTE_G6, 10, 150, 0);
  toneAC(NOTE_B5, 10, 150, 0);
  toneAC(NOTE_D6, 10, 250, 0);
  toneAC(NOTE_B5, 10, 150, 0);
  toneAC(NOTE_D6, 10, 500, 0);
  toneAC(0);
}

//---------------------------------------------------------------------------------------------//
// function shortBeep()
//
// plays a short beep asynchronously
//---------------------------------------------------------------------------------------------//
void shortBeep()
{
  toneAC(NOTE_A7, 10, 10, 1);
  toneAC(0);
}

//---------------------------------------------------------------------------------------------//
// function playAlarm()
//
// plays an alarm tone for a preset time
// stops if start/stop button is pressed
//---------------------------------------------------------------------------------------------//
void playAlarm()
{
  // track how long alarm has been playing
  unsigned long alarmTimer;
  // track time between alarm beeps
  unsigned long pauseTimer;
  // track if power button is pressed
  byte pressedOnOff = 0;
  
  // start timer
  alarmTimer = millis();
  // play the alarm until the timeout is exceeded or the power button is pressed
  while (((millis() - alarmTimer) < MAX_ALARM_TIME))
  {
    for (unsigned long freq = 500; freq <= 3000; freq += 10)
    {  
      toneAC(freq);
      delay(1);
      // test for power button
      if (onOffButton.update())
      {
        if (onOffButton.read() == LOW)
        {
          // set the stop flag
          pressedOnOff = 1;
          // exit the for loop
          break;
        }
      }
    }
    toneAC(0);
    // time the pause between beeps while tracking the start/stop button
    pauseTimer = millis();
    while (((millis() - pauseTimer) < ALARM_PAUSE_TIME))
    {
      // test for power button
      if (onOffButton.update())
      {
        if (onOffButton.read() == LOW)
        {
          // set the stop flag
          pressedOnOff = 1;
          // exit the while loop
          break;
        }
      }
    }
    if (pressedOnOff)
    {
      return;
    }
  }
}

//---------------------------------------------------------------------------------------------//
// function countBeeps()
//
// plays a count of short beeps
//---------------------------------------------------------------------------------------------//
void countBeep(byte count, unsigned long freq)
{
  for (int i = 1; i <= count; i++)
  {
    toneAC(freq, 10, 75, 0);
    delay(150);
  }
} 

//---------------------------------------------------------------------------------------------//
// function calculateTimeToBurn()
//
// calculate time to burn based on skin type and SPF
//---------------------------------------------------------------------------------------------//
void calculateTimeToBurn(byte index)
{
  minutesToBurn = skinTypeMinutesPerUVI[skinType] * uint16_t(pressesPerSPF[index]);
  // Serial.print("Minutes to burn: ");
  // Serial.println(minutesToBurn);
}

//---------------------------------------------------------------------------------------------//
// function doConfig()
//
// handles all the configuration stuff
//---------------------------------------------------------------------------------------------//
void doConfig()
{
  // reset the running time counter
  runningTimer = 0;
  // reset the WDT sleep count
  wdtSleepCount = 0;
  // spf index
  byte spfIndex = 1;
  // temporary skin type
  byte tempSkinType;
  // track if skin type button 1st press
  // this allows us to restart the skin type count on the first skin type button press
  // track button hold time
  byte skinTypeNotPressed = 1;
  // track the button hold time
  unsigned long buttonTime;
  
  // copy the skin type to the temp variable
  // this will be used to see if it was changed from power-up
  // to avoid writing it to the eeprom unnecessarily
  tempSkinType = skinType;
  
  // Serial.println("In config mode");
  
  // play the startup sound
  playStartup();
  // wait for user input
  // loop until setting timer expires
  // or the set button is held down
  // or the start button is pressed (only in the case of zero presses, ie no sunscreen)
  setTimer = millis();
  while ((millis() - setTimer) < SET_TIMEOUT)
  {
    // test for set button
    if (setButton.update())
    {
      if (setButton.read() == LOW)
      {
        // reset the set state timer
        setTimer = millis();
        // start timing
        buttonTime = millis();
        // loop until the button is released
        while(1)
        {
          if (setButton.update())
          {
            if (setButton.read() == HIGH)
            {
              // reset the set state timer
              setTimer = millis();
              // exit the button hold loop
              break;
            }
          }
        }
        // see how long the button was held down
        // if less than the long click time
        if (millis() - buttonTime < LONG_CLICK_MS)
        {
          // Serial.println("Short setButton");
          // count it as a press
          if (spfIndex < 6)
          {
            // increment the count
            spfIndex++;
            // short low beep
            toneAC(NOTE_C5, 10, 50, 1);
          }
          // roll over if greater than 5
          else
          {
            spfIndex = 1;
            // quick high beep
            toneAC(NOTE_C7, 10, 40, 0);
            // short low beep
            toneAC(NOTE_C5, 10, 50, 0);
          }
        }
        // otherwise it was a long press
        // move to the running state
        else
        {
          // Serial.println("Long setButton");
          // see if the skin type has been changed
          if (tempSkinType != skinType)
          {
            // make the skin type current
            skinType = tempSkinType;
            // Serial.print("Skin type: ");
            // Serial.println(skinType);
            // Serial.println("Writing skin type to eeprom");
            // store it in the eeprom
            EEPROM.write(EE_SKIN_TYPE, skinType);
          }
          
          // play back SPF count
          countBeep(spfIndex, NOTE_C5);
          // wait a second
          delay(1000);
          // play back skin type count
          countBeep(skinType, NOTE_C6);
          delay(1000);
          // calculate time to burn
          calculateTimeToBurn(spfIndex);
          // go the measuring state
          state = STATE_RUNNING;
          // turn off the UV sensor
          digitalWrite(UV_SENSOR_PWR_PIN, HIGH);
          // Serial.print("SPF index: ");
          // Serial.println(spfIndex);
          // Serial.print("Skin type: ");
          // Serial.println(skinType);
          // play the start tune
          playStart();
          // Serial.println("Leaving config mode");
          // enable the wdt
          enable_watchdog();
          // exit function
          return;
        }
      }
    }
    
    // test for skin button
    if (skinButton.update())
    {
      if (skinButton.read() == LOW)
      {
        // Serial.println("Short skinButton");
        // reset the set state timer
        setTimer = millis();
        // check if 1st button press
        if (skinTypeNotPressed)
        {
          // set the skin type to 1
          tempSkinType = 0;
          // unset the flag
          skinTypeNotPressed = 0;
        }
        // increment count if less than 6
        if (tempSkinType < 6)
        {
          // increment the count
          tempSkinType++;
          // short high beep
          toneAC(NOTE_C6, 10, 50, 1);
        }
        // roll over if greater than 5
        else
        {
          tempSkinType = 1;
          // quick low beep
          toneAC(NOTE_C5, 10, 40, 0);
          // short high beep
          toneAC(NOTE_C6, 10, 50, 0);
        }
      }
    }
  }
  // Serial.println("Set mode time limit exceeded");
  // if we drop out of the loop here
  // the state timer has expired
  // we will go to sleep to turn off
  state = STATE_POWER_OFF;
}

//---------------------------------------------------------------------------------------------//
// function doMeasure()
//
// takes a UV measurement
//---------------------------------------------------------------------------------------------//
void doMeasure()
{
  // turn on the UV sensor
  digitalWrite(UV_SENSOR_PWR_PIN, LOW);
  // allow the sensor to power up
  delay(200);
  // init the sensor
  uv.begin();
  delay(100);
  // take a reading
  float UVindex = uv.readUV();
  // turn off the UV sensor
  digitalWrite(UV_SENSOR_PWR_PIN, HIGH);
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;
  // Serial.print("UV index: ");
  // Serial.println(UVindex);
  // subtract time
  minutesToBurn = minutesToBurn - (1 * UVindex);
}

//---------------------------------------------------------------------------------------------//
// function getBandgap()
//
// measures the power input voltage
//---------------------------------------------------------------------------------------------//
int getBandgap(void) // Returns actual value of Vcc (x 100)
{
  
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // For mega boards
  const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
  // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR)| (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  
  #else
  // For 168/328 boards
  const long InternalReferenceVoltage = 1056L;  // Adjust this value to your boards specific internal BG voltage x1000
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
       
  #endif
  delay(50);  // Let mux settle a little to get a more stable A/D conversion
  // Start a conversion  
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
  // Scale the value
  int results = (((InternalReferenceVoltage * 1023L) / ADC) + 5L) / 10L; // calculates for straight line value
  

  return results;
  
}

//---------------------------------------------------------------------------------------------//
// function pin2_isr()
//
// interrupt handler for int0 pin2 interrupt
// causes sleep to disabled and int0 interrupt to be disabled
//---------------------------------------------------------------------------------------------//
void pin2_isr()
{
  sleep_disable();
  detachInterrupt(0);
}

//---------------------------------------------------------------------------------------------//
// function enable_watchdog()
//---------------------------------------------------------------------------------------------//
void enable_watchdog()
{
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP3; /* 4.0 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);
}

//---------------------------------------------------------------------------------------------//
// function disable_watchdog()
//---------------------------------------------------------------------------------------------//
void disable_watchdog()
{
  wdt_disable();
}

//---------------------------------------------------------------------------------------------//
// WDT interrupt handler
// 
// takes a UV reading every minute
//---------------------------------------------------------------------------------------------//
ISR(WDT_vect)
{
  wdtSleepCount++;
}

// puts the chip into sleep mode
//---------------------------------------------------------------------------------------------//
void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and 
     * wake up sources are available in which sleep modus.
     *
     * In the avr/sleep.h file, the call names of these sleep modus are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings 
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we 
     * choose the according 
     * sleep modus: SLEEP_MODE_PWR_DOWN
     *
     * Timer 2 overflow interrupt is only able to wake up the ATmega in PWR_SAVE
     * 
     */
    
    // save ADC state
    adcSettings = ADCSRA;
    // disable ADC
    ADCSRA = 0;
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
                             
    power_all_enable();      // turn the perhipherals on
    
    // put back the ADC state
    ADCSRA = adcSettings;
};
