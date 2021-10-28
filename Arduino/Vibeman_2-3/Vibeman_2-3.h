// Vibeman 2.3
// Created by Wolfstyle Lykin
// October 2021

/*
 * Vibeman is based on the Nogasm Rev.3 code
 * https://github.com/nogasm/nogasm
 * 
 * Vibeman improves upon Nogasm by providing real time data logging,
 * multiple vibration patterns, nRF24 wireless connection, and nRF24
 * slave mode operation.
 * 
 * The DIP switches of Nogasm are replaced with an nRF24L01+ and
 * carrier board, utilizing the SPI bus.  The base unit is capable
 * of operating in standalone, data reporting, and slave wireless
 * modes.  Configuration of advanced settings can be done through
 * the wireless station, or through USB serial.
 */

// ====- Libraries -====
#include <Encoder.h>
#include <EEPROM.h>
#include "FastLED.h"
#include "RunningAverage.h"
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <Wire.h>
#include "vibePatterns.h"

// ====- Compile Flags -====
#define HAS_RADIO             // Comment out if no nRF24 radio is installed
#define DEBUG                 // Enables USB Serial debug messages
//#define DEBUG_ONLY            // Disables non-debug USB Serial messages
//#define DEBUG_REMOTE          // Enables nRF24 debug messages

// ====- Hardware Definitions -====
// NeoPixels
#define PIX_NUM     13        // Number of NeoPixels in the string
#define PIX_PIN     17        // 5V buffered pin on Teensy LC, single wire data out to WS8212Bs
#define PIX_TYPE    WS2812B   // NeoPixel controller type
#define PIX_ORDER   GRB       // NeoPixel color data bit order
#define BRIGHTNESS  200       // Maximum NeoPixel color brightness

// Radio
#define ADDR1       "gboy1"   // Listening address
#define ADDR2       "gboy2"   // Transmitting address
#define CE          9         // Chip Enable pin
#define CSN         10        // Chip Select Not pin

// DIP Switches, only used if no Radio present
#define SW1PIN 12 //Dip switch pins, for setting software options without reflashing code
#define SW2PIN 11
#define SW3PIN 10
#define SW4PIN 9

// Encoder
#define ENC_R       5         //RGB pins of the encoder LED
#define ENC_G       4
#define ENC_B       3
#define ENC_SW      6         //Pushbutton on the encoder

// Motor
#define MOT_PIN     23        // Pin for the motor PWM control
#define MOT_FREQ    80        // PWM frequency for the motor PWM control
#define PI 3.14159268         // Pi for Cos tweening calculations

// Pressure Sensor Analog In
#define PRES_PIN    15        // Pressure sensor analog pin
#define DEFAULT_PLIMIT 600    // Default pressure limit setting

// ====- Timing -====
#define FREQ        60        // Device update frequency
#define SW_LONG     600       // Milliseconds to trigger a long button press
#define SW_VLONG    1500      // Milliseconds to trigger a very long button press
#define DEBOUNCE    50        // Milliseconds to ignore button changes

#define PERIOD (1000/FREQ)    // Milliseconds between device updates

#define RA_HIST_SECONDS 25    // Lifetime of running average for pressure
#define RA_FREQ 6             // Not really sure
#define RA_TICK_PERIOD (FREQ/RA_FREQ) // Also not really sure

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
