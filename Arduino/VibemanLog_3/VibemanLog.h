#ifndef __vibeman_h
#define __vibeman_h

// ====- Libraries -====
#include <DS1307RTC.h>
#include <TimeLib.h>
#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <FastLED.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "config.h"
#include "log.h"
#include "radio.h"
#include "console.h"

// ====- Defines -====
#define SD_FAT_TYPE 3 // Set the SD mode to FAT16/FAT32/exFAT

#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

#define PON SSD1306_WHITE   // OLED white color code
#define POFF SSD1306_BLACK  // OLED black color code

#define MAX_FILES 50        // Maximum number of files to load from the SD card
#define BUFFER_SIZE 50      // Maximum number of dataSet lines before writing to SD and flushing the buffer
#define GRAPH_SIZE 64       // Width of the graph panel

#define BTN_DEBOUNCE 50     // Time additional button presses are ignored
#define BTN_LONG 600        // Time before a long button press is registered
#define BTN_VLONG 1500      // Time before a very long button press is registered

// ====- Enum, struct setup -====
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

enum CURVE {
  LINEAR = 0,
  QUADRATIC,
  EASEOUT,
  EASEIN,
  COSINE,
  NA
};

struct dataRow {
  long ts;
  float motCur;
  uint8_t mode;
  int pressure;
  int avgPressure;
  uint8_t btn;
  int sensitivity;
};

// ====- Variable and object setup -====
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);   //Initialize screen object

#ifdef HAS_RADIO
RF24 radio(CE, CSN);           // CE, CSN not the other way around Snow!
const byte address[][6] = {ADDR1, ADDR2}; // Storage for Tx and Rx addresses
#endif

int sensitivity = 0;        // Orgasm detection sensitivity
bool motState = 0;          // Whether the motor is on or off
uint8_t motMax = 255;       // Maximum PWM for the motor
uint8_t motMin = 20;        // Minimum PWM for the motor
float motCur = 0;           // Current PWM for the motor
Btn btn = UP;               // Current state of the button
int rampTimeS = 30;         // Time in seconds for motor PWM transitions
Mode state = MANUAL;        // Current operation mode
int pressure = 0;           // Pressure sensor reading
int avgPressure = 0;        // From the pressure running average
char cmdTx[128] = "";       // Storage space for received commands
char cmdRx[128] = "";       // Storage space for sent commands
elapsedMillis rec = 0;      // Timestamp value
Mode mode = MANUAL;
bool recording = false;     // If values should be recorded
uint8_t pot = 0;

dataRow dSet[BUFFER_SIZE] = {{}};
uint8_t dNum = 0;
String dLabel[7] = {     "Timestamp",    "MotorCurrent",     "Mode",
                          "Pressure",     "AvgPressure",     "Button",
                          "Sensitivity"};

dataRow gSet[BUFFER_SIZE] = {{}};
String gLabel[7] = {     "Time",         "mCur",          "", "pCur",
                          "pAvg",         "Btn",          "Sen"};

String sLabel[7] = {      "Manual",       "Auto",         "Remote",           "Opt Speed",
                          "Opt Ramp",     "Opt Beep",     "Opt Pressure"};

// ====- Function declarations -====
void writeSerial();
int readRadio();
void print2digits(int number);
bool cardInit();
uint8_t fileOpen(int index = -1);
void fileClose();
void clearSet();
uint8_t dataSet();
bool dataWrite();
Btn btnCheck();
float tween(uint16_t curTick, uint16_t lastTick, CURVE curve = LINEAR);
void updateRec();
void parseRx();

#endif
