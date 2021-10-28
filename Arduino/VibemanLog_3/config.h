#ifndef __config_h
#define __config_h

// ====- Defines -====
#define HAS_DISP            // Flag for if the display is present
#define HAS_RADIO           // Flag for if an nRF24 radio is present
#define DEBUG               // Flag for debug serial writing
//#define DEBUGONLY           // Flag for only sending debug messages to Serial

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

// ====- Timing -====
#define SW_LONG     600       // Milliseconds to trigger a long button press
#define SW_VLONG    1500      // Milliseconds to trigger a very long button press
#define DEBOUNCE    50        // Milliseconds to ignore button changes

#endif
