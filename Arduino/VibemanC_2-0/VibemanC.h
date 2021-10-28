// ====- Libraries -====
#include <DS1307RTC.h>
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

// ====- Defines -====
#define HAS_DISP            // Flag for if the display is present
#define HAS_RADIO           // Flag for if an nRF24 radio is present
#define DEBUG               // Flag for debug serial writing
//#define DEBUGONLY           // Flag for only sending debug messages to Serial

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

#define BTNPIN 3      // Multifunction button
#define POTPIN A0     // Multifunction potentiometer
#define LEDPIN 4      // Status LED

#define SCREEN_WIDTH 128    // OLED width
#define SCREEN_HEIGHT 64    // OLED height
#define SCREEN_ADDRESS 0x3C // OLED i2c address
#define OLED_RST -1         // No dedicated OLED reset pin

#define ADDR1 "gboy2"       // Address for nRF radio
#define ADDR2 "gboy1"       // Address for nRF radio
#define CE 7                // Pin for radio Chip Enable
#define CSN 8               // Pin for radio Chip Select Not

#define PON SSD1306_WHITE   // OLED white color code
#define POFF SSD1306_BLACK  // OLED black color code

#define MAX_FILES 50        // Maximum number of files to load from the SD card
#define BUFFER_SIZE 50      // Maximum number of dataSet lines before writing to SD and flushing the buffer
#define GRAPH_SIZE 64       // Width of the graph panel

#define BTN_DEBOUNCE 50     // Time additional button presses are ignored
#define BTN_LONG 600        // Time before a long button press is registered
#define BTN_VLONG 1500      // Time before a very long button press is registered

// Enum setup
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

// Variable and object setup
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

float dSet[BUFFER_SIZE][10] = {{}};
uint8_t dNum = 0;
String dLabel[10] = {     "Timestamp",    "MotorState",       "MotorMax",         "MotorMin",
                          "MotorCurrent", "Button",           "RampTime",
                          "Pressure",     "PressureAverage",  "Sensitivity"};

float gSet[GRAPH_SIZE][10] = {{}};
uint8_t gNum = 0;
String gLabel[10] = {     "Time",         "mSte",         "mMax",             "mMin",
                          "mCur",         "Btn",          "Ramp",
                          "pCur",         "pAvg",         "Sens"};

String sLabel[7] = {      "Manual",       "Auto",         "Remote",           "Opt Speed",
                          "Opt Ramp",     "Opt Beep",     "Opt Pressure"};

// ====- Function delcarations -====
bool cardInit();
uint8_t fileOpen(int index = -1);
void fileClose();
void clearSet();
uint8_t dataSet();
bool dataWrite();
bool loadData();
void drawGraph();
void writeSerial(char &cmd);
void writeRadio(char &cmd);
int readSerial(char &buf);
int readRadio(char &buf);
void parseRx();
void parseData(char cmd[] = cmdRx);
