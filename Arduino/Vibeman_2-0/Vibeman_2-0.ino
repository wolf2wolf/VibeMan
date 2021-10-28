#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <FastLED.h>
#include <RunningAverage.h>

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

#define MOTPIN 13     // MOSFET for vibration motor PWM control
#define BTNPIN 3      // Multifunction button
#define POTPIN A0     // Multifunction potentiometer
#define LEDPIN 4      // Status LED

#define PIXPIN 9      // NeoPixel strip pin
#define PIXNUM 9      // Number of NeoPixels in the strip
#define PIXTYPE WS2812B // Controller type of NeoPixels
#define COLOR_ORDER GRB // Color order of NeoPixels
#define BRIGHTNESS 200  // Full brightness of NeoPixels

#define SCREEN_WIDTH 128    // OLED width
#define SCREEN_HEIGHT 64    // OLED height
#define SCREEN_ADDRESS 0x3C // OLED i2c address
#define OLED_RST -1         // No dedicated OLED reset pin

#define PON SSD1306_WHITE   // OLED white color code
#define POFF SSD1306_BLACK  // OLED black color code

#define MAX_FILES 50        // Maximum number of files to load from the SD card
#define BUFFER_SIZE 50      // Maximum number of dataSet lines before writing to SD and flushing the buffer
#define GRAPH_SIZE 64       // Width of the graph panel

#define BTN_DEBOUNCE 50     // Time additional button presses are ignored
#define BTN_LONG 600        // Time before a long button press is registered
#define BTN_VLONG 2500      // Time before a very long button press is registered

#define FREQ 60             // Main loop update Hz
#define PERIOD (1000/FREQ)  // ms between updates based on FREQ

#define RA_HIST_SECONDS 25  // Lifetime of running average for pressure
#define RA_FREQ 6           // Not really sure
#define RA_TICK_PERIOD (FREQ/RA_FREQ) // Also not really sure
#define DEFAULT_PLIMIT 600  // Default pressure change to trigger orgasm detection

#define HASDISP             // Flag for if the display is present
#define DEBUG               // Flag for debug serial writing

// Enum setup
enum Mode {
  MANUAL = 0,
  AUTO,
  OPT_SPEED,
  OPT_RAMPSPD,
  OPT_BEEP,
  OPT_PRES
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
//SdFatSdioEX Sd;             // Inbitialize the SD card object

CRGB pix[PIXNUM];           // Allocate array for NeoPixel colors

RunningAverage raPressure(RA_FREQ*RA_HIST_SECONDS);   // Initialize the running average for the pressure

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

float dSet[BUFFER_SIZE][10] = {{}};
uint8_t dNum = 0;
String dLabel[10] = {     "Timestamp",    "MotorState",   "MotorMax",         "MotorMin",
                          "MotorCurrent", "Button",           "RampTime",
                          "Pressure",     "PressureAverage",  "Sensitivity"};

float gSet[GRAPH_SIZE][10] = {{}};
uint8_t gNum = 0;
String gLabel[10] = {     "Time",         "mSte",         "mMax",             "mMin",
                          "mCur",         "Btn",              "Ramp",
                          "pCur",         "pAvg",             "Sens"};

//File file;                  // File for SD read/write

// Functions
void beep(int f1, int f2, int f3) {
  if(motCur > 245) analogWrite(MOTPIN, 245); //make sure the frequency is audible
  else if(motCur < 10) analogWrite(MOTPIN, 10);
  
  analogWriteFrequency(MOTPIN, f1);
  delay(250);
  analogWriteFrequency(MOTPIN, f2);
  delay(250);
  analogWriteFrequency(MOTPIN, f3);
  delay(250);
  analogWriteFrequency(MOTPIN, 440);
  
  analogWrite(MOTPIN,motCur);                 // Return motor to previous speed)
}

bool cardInit() {
  // Open card
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50)))) return false;
  return true;
}

uint8_t fileOpen(int index = -1) {
  // If index -1, increment file and write header
  // If index not -1, open file
  bool newfile = false;
  if (!sd.exists("SessionLog.csv"))
    newfile = true;
    
  if (!file.open("SessionLog.csv", O_RDWR | O_CREAT)) return 0;
  if (newfile) {
    for (uint8_t i = 0; i < 10; i++) {
      file.print(dLabel[i]);
      if (i < 9) file.print(',');
    }
    if (!file.print(F("\r\n"))) Serial.println("File write failed"); else Serial.println("File write successful");
  }
  if (newfile)
    return 2;

  return 1;
}

void fileClose() {
  file.close();
}

void clearSet() {
  // erase dataSet, set index to 0
  memset (dSet, 0, sizeof(dSet));
  dNum = 0;
}

uint8_t dataSet() {
  // Add data set to buffer
  // Add data to graphSet array, pop if necessary
  // Return free space in buffer
  float tmpRow[10] = {millis(), motState, motMax, motMin, motCur, btn, rampTimeS, pressure, avgPressure, sensitivity};
  memcpy(dSet[dNum++], tmpRow, sizeof(tmpRow));
  //dSet[dNum++] = tmpRow;
  return 0;
}

bool dataWrite() {
  // Write dataSet to SD card
  
  return false;
}

bool loadData() {
  // load data from current SD file to graphSet, working backwards until header or graphSet full

  return false;
}

void drawGraph() {
  // Working from end of graphSet, draw data to graph
}

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<PIXTYPE, PIXPIN, COLOR_ORDER>(pix, PIXNUM).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  pinMode(MOTPIN, OUTPUT);
  pinMode(BTNPIN, INPUT_PULLUP);
  pinMode(POTPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(PIXPIN, OUTPUT);

  analogReadRes(12);
  analogReadAveraging(32);

  digitalWrite(MOTPIN, LOW);
  digitalWrite(LEDPIN, LOW);
  //display.clearDisplay();
  // Display some sort of logo?
  display.display();
  for (uint8_t i = 0; i < PIXNUM; i++) {
    if (i > 0) pix[i-1] = 0x000000;
    pix[i] = 0xffffff;
    FastLED.show();
    delay(250);
  }
  pix[PIXNUM-1] = 0x000000;
  FastLED.show();
  digitalWrite(LEDPIN, HIGH);

  if (cardInit()) {
    Serial.println("SD card initialized");
    if (fileOpen()) {
      Serial.println("File opened");
      fileClose();
    }else Serial.println("Failed to open file");
  }else Serial.println("Failed to initialize SD card");

  Serial.println("Setup complete.");
  display.clearDisplay();
  // Display UI
  display.display();
  beep(1047, 1396, 2093);
}

void loop() {
  
}
