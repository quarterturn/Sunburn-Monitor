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
  
  TODO:
  function to play startup sound
  function to play shutdown sound
  wdt configuration for 4 second timer
  sleep function
  uv read function
  alarm sound function
  bounce library for button presses
 ****************************************************/

// for i2c
#include <Wire.h>
// library for Si1145/Si1135
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
#define START_STOP_PIN 3
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
// track total running time in minutes
// we add a minute each time we take a sample from the UV sensor
uint16_t runningTimer = 0;
// flag if start/stop is allowed
byte readyToStart = 0;


// skin type - default to 3
uint8_t skinType = 3;
// skin type lookup table for sunburn minutes/UVI
static uint16_t skinTypeMinutesPerUVI[6] = {0, 67, 100, 200, 400, 500};
     
// minutes until sunburn
float minutesToBurn;

// watchdog timer sleep count
volatile byte wdtSleepCount = 0;

// spf for sunscreen
// map spf set button presses to spf
static uint8_t pressesPerSPF[5][2] = {1, 10, 15, 30, 45};                             

// create a 'uv' object based on Adafruit_SI1145
Adafruit_SI1145 uv = Adafruit_SI1145();

// set up buttons
Bounce onOffButton = Bounce();
Bounce startStopButton = Bounce();
Bounce setButton = Bounce();

//---------------------------------------------------------------------------------------------//
// setup
//---------------------------------------------------------------------------------------------//
void setup()
{
  // set up the UV sensor power pin
  // set it to a low output
  // setup runs when the battery is inserted
  // we go to sleep at the end of setup, so we want the UV sensor off
  // we will turn it back on when we leave sleep
  pinMode(UV_SENSOR_PWR_PIN, OUTPUT);
  digitalWrite(UV_SENSOR_PWR_PIN, LOW);
  
  // set up the buttons
  // on/off
  pinMode(ON_OFF_PIN, INPUT);
  digitalWrite(ON_OFF_PIN, HIGH);
  onOffButton.attach(ON_OFF_PIN);
  onOffButton.interval(DEBOUNCE_MS);
  // start/stop
  pinMode(START_STOP_PIN, INPUT);
  digitalWrite(START_STOP_PIN, HIGH);
  startStopButton.attach(START_STOP_PIN);
  startStopButton.interval(DEBOUNCE_MS);
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
    // the skin type will default to 3
    EEPROM.write(EE_SKIN_TYPE, 3);
  }
  
//  Serial.begin(9600);
  
//  Serial.println("SI1135 test");
//  
//  if (! uv.begin()) {
//    Serial.println("Didn't find Si1135");
//    while (1);
//  }
//
//  Serial.println("OK!");
  
  // play startup so the user knows the battery is good
  playStartup();
  
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
      doConfig();
      // if state is power off skip the rest of this case section
      if (state == STATE_POWER_OFF)
      {
        break;
      }
      else
      {
        // enable the WDT
        WDTCSR |= (1 << WDIE); 
        // go to sleep
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
          // detach the WDT interrupt
          // disable the WDT
          WDTCSR |= (0 << WDIE);
          // switch to power down state
          state = STATE_POWER_OFF;
        }
        // do measuring and testing
        else
        {
          // short beep for testing
          shortBeep();       
          // do stuff here for measuring
          doMeasure();
          // check if we are out of time
          if (minutesToBurn <= 0)
          {
            // detatach the WDT interrupt
            // disable the WDT
            WDTCSR |= (0 << WDIE); 
            // go to state completed
            state = STATE_COMPLETED;
          }
          // otherwise reset the WDT sleep count
          // and go back to sleep
          else
          {
            // reset the WDT sleep count
            wdtSleepCount = 0;
            // go back to sleep
            sleepNow();
          }
        }
      }
      break;
      
    // completed state
    case STATE_COMPLETED:
      // do stuff here once sunburn limit is reached
      playAlarm();
      // go to power down state
      state = STATE_POWER_OFF;
      break;
      
    // power off state (deep sleep)
    case STATE_POWER_OFF:
      // turn off the uv sensor
      digitalWrite(UV_SENSOR_PWR_PIN, LOW);
      // reset the timer
      runningTimer = 0;
      // play the shutdown sound
      playShutdown();
      // attach interrupt
      attachInterrupt(0, pin2_isr, LOW);
      // go to sleep here
      sleepNow();
      // after waking up, go back to config state
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
  toneAC(NOTE_D6, 10, 150, 0);
  toneAC(NOTE_B5, 10, 150, 0);
  toneAC(NOTE_D6, 10, 150, 0);
  toneAC(0);
}

//---------------------------------------------------------------------------------------------//
// function shortBeep()
//
// plays a short beep asynchronously
//---------------------------------------------------------------------------------------------//
void shortBeep()
{
  toneAC(NOTE_C6, 10, 100, 1);
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
  // track if start/stop is pressed
  byte pressedStop = 0;
  
  // start timer
  alarmTimer = millis();
  // play the alarm until the timeout is exceeded or the star/stop button is pressed
  while (((millis() - alarmTimer) < MAX_ALARM_TIME) || (pressedStop))
  {
    for (unsigned long freq = 500; freq <= 3000; freq += 10)
    {  
      toneAC(freq);
      delay(1);
      // test for start/stop button
      if (startStopButton.update())
      {
        if (startStopButton.read() == LOW)
        {
          // set the stop flag
          pressedStop = 1;
          // exit the for loop
          break;
        }
      }
    }
    toneAC(0);
    // time the pause between beeps while tracking the start/stop button
    while (((millis() - pauseTimer) < ALARM_PAUSE_TIME) || (pressedStop))
    {
      // test for start/stop button
      if (startStopButton.update())
      {
        if (startStopButton.read() == LOW)
        {
          // set the stop flag
          pressedStop = 1;
          // exit the while loop
          break;
        }
      }
    }
  }
}
  

//---------------------------------------------------------------------------------------------//
// function resetBeep()
//
// plays two descending beeps to indicate set count has reset to the start
//---------------------------------------------------------------------------------------------//
void resetBeep()
{
  toneAC(NOTE_C6, 10, 100, 0);
  toneAC(NOTE_C5, 10, 100, 0);
  toneAC(0);
}

//---------------------------------------------------------------------------------------------//
// function countBeeps()
//
// plays a count of short beeps
//---------------------------------------------------------------------------------------------//
void countBeep(byte count)
{
  for (int i = 1; i <= count; i++)
  {
    toneAC(NOTE_C6, 10, 75, 0);
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
}

//---------------------------------------------------------------------------------------------//
// function doConfig()
//
// handles all the configuration stuff
//---------------------------------------------------------------------------------------------//
void doConfig()
{
  // button press count
  byte presses = 0;
  // track button hold time
  unsigned long buttonTime;
  // flag if OK to start
  // accept start button at zero presses, or after long press of set
  readyToStart = 1;
  
  // play the startup sound
  playStartup();
  // wait for user input
  // loop until setting timer expires
  // or the set button is held down
  // or the start button is pressed (only in the case of zero presses, ie no sunscreen)
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
              // exit the button hold loop
              break;
            }
          }
        }
        // see how long the button was held down
        // if less than the long click time
        if (millis() - buttonTime < LONG_CLICK_MS)
        {
          // count it as a press
          if (presses < 5)
          {
            // increment the count
            presses++;
            // block being able to start without holding down set
            readyToStart = 0;
          }
          // roll over if greater than four
          else
          {
            presses = 0;
          }
        }
        // otherwise it was a long press
        else
        {
          // if greater than zero presses, beep back the number of presses to confirm
          if (presses > 0)
          {
            countBeep(presses);
          }
          // allow start
          readyToStart = 1;
        }
      }
    }
    
    // test for start button and readyToStart
    if (startStopButton.update() && readyToStart)
    {
      if (startStopButton.read() == LOW)
      {
        // calculate time to burn
        calculateTimeToBurn(presses);
        // go the measuring state
        state = STATE_RUNNING;
        // turn on the UV sensor
        digitalWrite(UV_SENSOR_PWR_PIN, HIGH);
        // exit function
        return;
      }
    }
  }
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
  // take a reading
  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;
  // subtract time
  minutesToBurn = minutesToBurn - (1 * UVindex);
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
// function setup_watchdog()
// sets up the watchdog timer but does not enable it
// set up a two second timer
//---------------------------------------------------------------------------------------------//
void setup_watchdog()
{
  // reset status flag
  MCUSR &= ~(1 << WDRF);
  // enable configuration change
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // set the prescaler to 8 -- two seconds
  WDTCSR = (0 << WDP0) | (0 << WDP1) | (0 << WDP2) | (1 << WDP3); 
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
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin 

    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
                             
    power_all_enable();      // turn the perhipherals on
};
