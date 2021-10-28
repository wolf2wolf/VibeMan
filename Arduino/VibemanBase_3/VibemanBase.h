#ifndef __vibeman_h
#define __vibeman_h

// ====- Libraries -====
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include "RunningAverage.h"
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <Wire.h>
#include "config.h"
#include "patterns.h"
#include "radio.h"
#include "console.h"

// ====- EEPROM Addresses -====
//128b available on teensy LC
#define BEEP_ADDR         1
#define MAX_SPEED_ADDR    2
#define SENSITIVITY_ADDR  3
//#define RAMPSPEED_ADDR    4 //For now, ramp speed adjustments aren't implemented

// ====- Enumeration Setup -====
enum Mode {
  MANUAL = 0,
  AUTO = 1,
  REMOTE = 2,
  OPT_SPEED = 3,
  OPT_RAMPSPD = 4,
  OPT_BEEP = 5,
  OPT_PRES = 6
};

enum Btn {
  UP = 0,
  SHORT = 1,
  LONG = 2,
  VLONG = 3,
  DOWN = 4
};

// ====- Variable and object setup -====
CRGB pix[PIX_NUM];           // Allocate array for NeoPixel colors
Encoder myEnc(8, 7);         //Quadrature inputs on pins 7,8
RunningAverage raPressure(RA_FREQ*RA_HIST_SECONDS);   // Initialize the running average for the pressure

#ifdef HAS_RADIO
RF24 radio(CE, CSN);           // CE, CSN not the other way around Snow!
const byte address[][6] = {ADDR1, ADDR2}; // Storage for Tx and Rx addresses
#endif

int sensitivity = 0;        // Orgasm detection sensitivity
bool motState = 0;          // Whether the motor is on or off
uint8_t motMax = 255;       // Maximum PWM for the motor
uint8_t motMin = 20;        // Minimum PWM for the motor
float motCur = 0;           // Current PWM for the motor
uint16_t motTick = 0;       // Animation tick for motor vibration patterns
uint8_t motStep = 0;        // Animation step for the motor vibration patterns
Btn btn = UP;               // Current state of the button
int rampTimeS = 30;         // Time in seconds for motor PWM transitions
Mode state = MANUAL;        // Default operation mode
int pressure = 0;           // Pressure sensor reading
int avgPressure = 0;        // From the pressure running average
int pLimit = DEFAULT_PLIMIT;  // Limit in change of pressure before orgasm is detected
elapsedMillis tickUpdate = 0; // Animation frame counter

char cmd[128] = "";          // Serial and radio command character array

// Serial data identifiers
String sLabel[10] = {     "TS", "MS", "MX", "MN",
                          "MC", "MD", "RT",
                          "PS", "PA", "SN"};
                          // Timestamp, Motor State, Motor Max
                          // Motor Current, Mode, Ramp Time,
                          // Pressure, Pressure Average, Sensitivity

uint8_t curPattern = 0;

// ====- Function Declarations -====
void beep(int f1, int f2, int f3);
void showKnobRGB(const CRGB& rgb);
void drawCursor3(int pos, CRGB C1, CRGB C2, CRGB C3);
void drawCursor(int pos, CRGB C1);
void drawBars3(int pos, CRGB C1, CRGB C2, CRGB C3);
int encLimitRead(int eMin, int eMax);
void runManual();
void runAuto();
void runRadio();
void runOptSpeed();
void runOptRamp();
void runOptBeep();
void runOptPres();
Btn btnCheck();
void runUpdate(Mode state);
Mode setState(Btn btnState, Mode state);
void writeSerial(char &cmd);
void writeRadio(char &cmd);
int readSerial(char &buf);
int readRadio(char &buf);
void parseCmd();
void parseData();
void updateMotor();
float tween(uint16_t curTick, uint16_t lastTick, CURVE curve = LINEAR);

#endif
