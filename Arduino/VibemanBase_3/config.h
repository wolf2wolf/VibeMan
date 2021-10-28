#ifndef __config_h
#define __config_h

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

// Pressure Sensor Analog In
#define PRES_PIN    15        // Pressure sensor analog pin
#define DEFAULT_PLIMIT 600    // Default pressure limit setting

// ====- Timing -====
#define FREQ        60        // Device update frequency
#define PERIOD      (1000/FREQ) // Milliseconds between device updates
#define SW_LONG     600       // Milliseconds to trigger a long button press
#define SW_VLONG    1500      // Milliseconds to trigger a very long button press
#define DEBOUNCE    50        // Milliseconds to ignore button changes

#define RA_HIST_SECONDS 25    // Lifetime of running average for pressure
#define RA_FREQ 6             // Not really sure
#define RA_TICK_PERIOD (FREQ/RA_FREQ) // Also not really sure

#endif
