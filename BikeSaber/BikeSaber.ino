///
/// @mainpage	blinkLed
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Justin Gregg
/// @author		Rodentia
/// @date		7/18/18 5:45 PM
/// @version	<#version#>
///
/// @copyright	(c) Justin Gregg, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		BikeSaber.ino
/// @brief		Main sketch for Burning Man BikeSabers
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Justin Gregg
/// @author		Rodentia
/// @date		7/18/18 5:45 PM
/// @version	<#version#>
///
/// @copyright	(c) Justin Gregg, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE


// Set parameters


// Include application, user and local libraries
#include "RH_RF69.h"
#include "Adafruit_NeoPixel.h"

// Accelerometer 
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
// Accelerometer Hardware SPI Comms
//#define LIS3DH_CS 10
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);

// Accelerometer via I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();


// Define structures and classes


// Define variables and constants
const int lessLight = 0;  // use this for longer strings. It will disable every other LED on brighter programs to limit power.
const int testMode = 0;     // If testing with just one BikeSaber, use this mode which: moves to the next program sequentially
const int transmitMode = 1;  // use this for BikeSabers that we only want to recieve, but not vote.
static int useAccel = 1; // we will set this to 0 if we can't find accel

// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Setup

// ***************************************************************************
// Stuff for RFM69
// ***************************************************************************
// Add setup code
#define RF69_FREQ 915.0


//#if defined (ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
//#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// ***************************************************************************
// Stuff for LED string test
// ***************************************************************************
#define NUMPIXELS 130  // For Bike Whips
//#define NUMPIXELS 50 // For Bike Wheels
#define PIXEL_PIN 6
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800); // for 8mm NeoPixels

// ***************************************************************************
// Stuff for timer test
// ***************************************************************************
#define LED_PIN 13

//#define CPU_HZ 48000000
//#define TIMER_PRESCALER_DIV 64
//uint32_t compareValue=0;

//void startTimerMs(int millsec);
//void startTimerSec(int seconds);
//void _setTimer(int millsec, unsigned int scaler);
//void TC3_Handler();
//
void setup()
{
    Serial.begin(115200);
    //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
     if (testMode >= 1) {while (!Serial);}     // will pause Zero, Leonardo, etc until serial console opens


/////// Setup Radio RFM69
    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    Serial.println("Feather RFM69 TX Test!");
    Serial.println();
    
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    
    if (!rf69.init()) {
        Serial.println("RFM69 radio init failed");
        while (1);
    }
    Serial.println("RFM69 radio init OK!");
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69.setFrequency(RF69_FREQ)) {
        Serial.println("setFrequency failed");
    }
    
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    
    // The encryption key has to be the same as the one in the server
    uint8_t key[] = { 0xD3, 0xAD, 0x00, 0xB3, 0xB3, 0xF0, 0x07, 0x08,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    pinMode(LED, OUTPUT);
    
    Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");


///////// setup accelerometer stuff
    Serial.println("LIS3DH test!");
    
    if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
      Serial.println("Couldnt start accel! Continuing without it.");
      useAccel = 0;
    } else {
      Serial.println("LIS3DH found!");
      
      lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
      
      Serial.print("Range = "); Serial.print(2 << lis.getRange());  
      Serial.println("G");
    }

    
////// Setup the NeoPixel string
    strip.begin(); // This initializes the NeoPixel library.
    strip.show(); // start with everything off
    
}

// Utilities

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return strip.Color(200 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 200 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 200 - WheelPos * 3, 0);
}

void setPixel(int Pixel, byte red, byte green, byte blue) {  // This utility makes it easier to set a pixel to color provided RGB
    strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

void setAll(byte red, byte green, byte blue) {  // This utility makes it easier to set all pixels to a color
  for(int i = 0; i < strip.numPixels() + strip.numPixels(); i++ ) {
    setPixel(i, red, green, blue); 
  }
   strip.show();
}

// helper to fade current strip setting towards black
void fadeToBlack(int ledNo, byte fadeValue) {
    uint32_t oldColor;
    uint8_t r, g, b;
    int value;
    
    oldColor = strip.getPixelColor(ledNo);
    r = (oldColor & 0x00ff0000UL) >> 16;
    g = (oldColor & 0x0000ff00UL) >> 8;
    b = (oldColor & 0x000000ffUL);
    
    r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
    g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
    b=(b<=10)? 0 : (int) b-(b*fadeValue/256);
    
    strip.setPixelColor(ledNo, r,g,b);
}


// Functions for LED programs
// All LED functions should store state in these global variable
// These variables get reset after changing programs
// If a global vaiable is added for LED program tracking, add it to the reset function, too
int16_t g_LedProgramCurrentPixel = 0;
uint32_t g_LedProgramColor = 0;
uint8_t g_LedProgramState = 0;

/***********************************************************************/
// Clear all the static program states so that we can have all programs
// start in a fresh state. This will help keep things in visual sync
/***********************************************************************/
void resetAllLedProgramStates(){
    setAll (0,0,0); // clear all LEDs by setting them to off
    g_LedProgramCurrentPixel = 0;
    g_LedProgramColor = 0;
    g_LedProgramState = 0;
}

/***********************************************************************/
// Color Wipe
// Fill the dots one after the other with a color
// Then wipe them all off again
// concept from:
// https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/

/***********************************************************************/
void colorWipe(uint32_t c) {
    // use g_LedProgramState for wipe on/off state
    switch(g_LedProgramState){
        default:
            g_LedProgramState = 0;
            g_LedProgramCurrentPixel = 0;
            // fall through
        case 0: // wipe color on
            strip.setPixelColor(g_LedProgramCurrentPixel, c);
            strip.show();
            break;
        case 1: // wipe color off
            strip.setPixelColor(g_LedProgramCurrentPixel, 0);
            strip.show();
            break;
    }
    
    // move to the next pixel
    g_LedProgramCurrentPixel++;
    
    // if we've filled the strip, flip on/off and refill
    // add a few pixels over the strip length to make it stay lit/off for a bit
    if(g_LedProgramCurrentPixel >= strip.numPixels()+25){
        g_LedProgramState++;
        g_LedProgramCurrentPixel = 0;
    }
}

// helper function to do a wipe with RGB params
void colorWipe(byte red, byte green, byte blue) {
    colorWipe(strip.Color(red, green, blue));
}

/***********************************************************************/
// Police Mode
// Flash Red-Blue on the full string
/***********************************************************************/
uint16_t policeMode() {
    uint16_t delayForNextUpdateMs = 100;
    switch(g_LedProgramState){
        default:
        case 0: // red color wipe
            g_LedProgramState++;
            for(int i=0; i < (strip.numPixels()); i++){
                if ( lessLight == 1 ) { i=i+1;}   //turn off every other pixel
                strip.setPixelColor(i, strip.Color(0, 200, 0));
            }
            strip.show();
            delayForNextUpdateMs = 100;
            break;
        case 1: // all off
            g_LedProgramState++;
            setAll(0,0,0);
            delayForNextUpdateMs = 10;
            break;
        case 2: // blue color wipe
            g_LedProgramState++;
            for(int i=0; i < (strip.numPixels()); i++){
                if ( lessLight == 1 ) { i=i+1;}   //turn off every other pixel
                strip.setPixelColor(i, strip.Color(0, 0, 200));
            }
            strip.show();
            delayForNextUpdateMs = 100;
            break;
        case 3: // all off
            g_LedProgramState = 0;
            setAll(0,0,0);
            delayForNextUpdateMs = 10;
            break;
    }
    
    return delayForNextUpdateMs;
}

/***********************************************************************/
// China Police Mode with full or half string modes
// Fast strobe on red; pause; fast strobe on blue; pause
/***********************************************************************/
uint16_t policeChinaMode2(int strobeCount, int flashDelay, int endPause, bool halfString) {
    // use g_LedProgramCurrentPixel as a strobe counter
    // use g_LedProgramState for state machine state
    // use g_LedProgramColor for current color
    uint16_t delayForNextUpdateMs = 10;
    const uint32_t colorsForFlashing[2] = {strip.Color(0, 150, 0), strip.Color(0, 0, 255)};
    switch(g_LedProgramState){
        default:
        case 0:
            // initial or invalid state; set things back to default and start over
            g_LedProgramState = 0;
            g_LedProgramCurrentPixel = 0;
            g_LedProgramColor = colorsForFlashing[0];  // dim red
            g_LedProgramState++;
            // fall through
        case 1: // strobe color on
            // create the color to strobe on
            if(halfString){
                // for half string flashing, use one side for red and one side for blue
                if(g_LedProgramColor == colorsForFlashing[0]){
                    strip.fill(g_LedProgramColor, 0, (NUMPIXELS/2));
                } else {
                    strip.fill(g_LedProgramColor, (NUMPIXELS/2), NUMPIXELS);
                }
            }
            else {
                // for full string, just fill the full string with the current color
                strip.fill(g_LedProgramColor,0, NUMPIXELS);
            }
            strip.show();
            
            // setup for strobe delay before turning off
            delayForNextUpdateMs = flashDelay;
            
            // go to the next state
            g_LedProgramState++;
            break;
        case 2: // strobe color off
            // turn off all LEDs
            strip.clear();
            strip.show();
            
            // increment the number of strobe flashes we've done
            g_LedProgramCurrentPixel++;
            
            // if we've done the desired number of flashes, move the state forward
            if(g_LedProgramCurrentPixel < strobeCount){
                // keep strobing
                // set up the delay for the strobe effect
                delayForNextUpdateMs = flashDelay;
                g_LedProgramState = 1;
            } else {
                // done strobing on the current color
                g_LedProgramCurrentPixel = 0; // reset strobe counter
                g_LedProgramState++;
                
                // set up the delay for going to the next color
                delayForNextUpdateMs = endPause;
                
                // change the color
                if (g_LedProgramColor == colorsForFlashing[0]){
                    g_LedProgramColor = colorsForFlashing[1];  // blue
                } else {
                    g_LedProgramColor == colorsForFlashing[0]; // dim red
                }
            }
            break;
        case 3: // end pause between colors
            delayForNextUpdateMs = endPause;
            g_LedProgramState = 1;
            break;
    } // end switch state machine
    
    return delayForNextUpdateMs;
}

/***********************************************************************/
// Rainbow
// Fill the LEDs with a moving rainbow
/***********************************************************************/
void rainbow() {
    // set each LED to an increment color from wheel; this makes a rainbow
    for(uint16_t i=0; i<strip.numPixels(); i=i+1) {
        if ( lessLight == 1 ) { strip.setPixelColor(i, 0); i=i+1;}   //turn off every other pixel
        strip.setPixelColor(i, Wheel((i+g_LedProgramColor) & 255));
    }
    
    // show the rainbow
    strip.show();
    
    // move to the next initial color
    g_LedProgramColor++;
    if(g_LedProgramColor > 255) g_LedProgramColor = 0;
}

/***********************************************************************/
// Rainbow cycle
// Slightly different from rainbow, this makes the rainbow equally
// distributed throughout the strip
/***********************************************************************/

void rainbowCycle() {
    for(uint16_t i=0; i< strip.numPixels(); i=i+1) {
        if ( lessLight == 1 ) { strip.setPixelColor(i, 0); i=i+1;}   //turn off every other pixel
        strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + g_LedProgramColor) & 255));
    }

    // show the rainbow
    strip.show();
    
    // move to the next initial color
    g_LedProgramColor++;
    if(g_LedProgramColor > 256*5) g_LedProgramColor = 0;
}

/***********************************************************************/
// Theater chase
// Theatre-style crawling lights.
/***********************************************************************/
void theaterChase(uint32_t color) {
    // use g_LedProgramCurrentPixel for current pixel position
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        //turn off every third pixel from the previous run
        strip.setPixelColor(i+g_LedProgramCurrentPixel-1, 0);
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel, color);    //set every third pixel
    }
    strip.show();

    // move the starting pixel forward one
    g_LedProgramCurrentPixel++;
    
    // reset to first pixel after moving 3
    if(g_LedProgramCurrentPixel >= 3) g_LedProgramCurrentPixel = 0;

}

/***********************************************************************/
//Theatre-style crawling lights with rainbow effect
/***********************************************************************/
void theaterChaseRainbow() {
    // use g_LedProgramCurrentPixel for current pixel position
    // use g_LedProgramColor for rainbox color position
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel-1, 0);        //turn every third pixel off
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel, Wheel( (i+g_LedProgramColor) % 255));    //turn every third pixel on
    }
    strip.show();

    // move the starting pixel forward one
    g_LedProgramCurrentPixel++;
    // reset to first pixel after moving 3
    if(g_LedProgramCurrentPixel >= 3) g_LedProgramCurrentPixel = 0;
    
    // increment the initial color position
    g_LedProgramColor++;
    if(g_LedProgramColor > 255) g_LedProgramColor = 0;
}


/***********************************************************************/
// Meteor rain
// make it rain glowing rocks
/***********************************************************************/
uint16_t meteorRain(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay) {
    // use g_LedProgramCurrentPixel for meteor position
    // use g_LedProgramState for initialization flag
    
    // initialize or reset the initial meteor position
    if(g_LedProgramState == 0){
        g_LedProgramCurrentPixel = strip.numPixels() + (strip.numPixels()/2);
        g_LedProgramState++;
    }
    
    // reset the meteor position after it shoots through
    if(g_LedProgramCurrentPixel <= (0 - meteorSize - meteorTrailDecay)){
        g_LedProgramCurrentPixel = strip.numPixels() + (strip.numPixels()/2);
    }
    
    
    // fade brightness all LEDs one step
    for(int j=0; j< strip.numPixels(); j++) {
        if( (!meteorRandomDecay) || (random(10)>5) ) {
            fadeToBlack(j, meteorTrailDecay );
        }
    }
    
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
        if( ( g_LedProgramCurrentPixel - j < strip.numPixels() ) && ( g_LedProgramCurrentPixel - j >= 0 ) ) {
            strip.setPixelColor(g_LedProgramCurrentPixel - j, red, green, blue);
        }
    }
    
    strip.show();
    g_LedProgramCurrentPixel--;
    
}

/***********************************************************************/
// Strobe
// Flashy all the string with one color
/***********************************************************************/
uint16_t Strobe(byte red, byte green, byte blue, int strobeCount, int flashDelay, int endPause){
    // use g_LedProgramState for on/off/pause state
    // use g_LedProgramCurrentPixel as a strobe counter
    
    uint32_t strobeColor = strip.Color(red, green, blue);
    uint16_t delayForNextUpdateMs = flashDelay;
    
    switch(g_LedProgramState){
        default:
        case 0: // initialize
            g_LedProgramCurrentPixel = 0;
            g_LedProgramState++;
            // fall through
        case 1: // turn all on
            strip.fill(strobeColor, 0, strip.numPixels());
            strip.show();
            delayForNextUpdateMs = flashDelay;
            g_LedProgramState++;
            break;
        case 2: // turn all off
            strip.clear();
            strip.show();
            
            // set next state to turn things on
            g_LedProgramState = 1;
            
            // increment the number of strobe flashes we've done
            g_LedProgramCurrentPixel++;
            // if we've done the desired number of flashes, give a longer pause
            if(g_LedProgramCurrentPixel < strobeCount){
                // keep strobing
                // set up the delay for the strobe effect
                delayForNextUpdateMs = flashDelay;
            } else {
                // reset the strobe counter
                g_LedProgramCurrentPixel = 0; // reset strobe counter
                
                // set up the delay for going to the next color
                delayForNextUpdateMs = endPause;
            }
            break;
    }
    
    return delayForNextUpdateMs;
}

/***********************************************************************/
// Sutro emulator?
/***********************************************************************/
void Sutro(){
    // use g_LedProgramCurrentPixel for current position tracking
    
    const uint32_t RedColor = strip.Color(0, 200, 0);
    const uint32_t WhiteColor = strip.Color(150, 100, 200);
    g_LedProgramCurrentPixel++;
    
    if (g_LedProgramCurrentPixel > strip.numPixels()) g_LedProgramCurrentPixel = 1;
    
    strip.fill(WhiteColor, 0+g_LedProgramCurrentPixel-int(strip.numPixels() *3/4),
               0+g_LedProgramCurrentPixel-int(strip.numPixels()/2));
    strip.fill(RedColor, 0+g_LedProgramCurrentPixel-int(strip.numPixels()/2),
               0+g_LedProgramCurrentPixel-int(strip.numPixels()/4));
    strip.fill(WhiteColor, 0+g_LedProgramCurrentPixel-int(strip.numPixels()/4),
               0+g_LedProgramCurrentPixel);
    strip.fill(RedColor, 0+g_LedProgramCurrentPixel,
               int(strip.numPixels()/4)+g_LedProgramCurrentPixel); // bottom 1/4 is red
    strip.fill(WhiteColor, int(strip.numPixels()/4)+g_LedProgramCurrentPixel,
               int(strip.numPixels()/2)+g_LedProgramCurrentPixel); // bottom 1/4-1/2 is white
    strip.fill(RedColor, int(strip.numPixels()/2)+g_LedProgramCurrentPixel,
               int(strip.numPixels() * 3/4)+g_LedProgramCurrentPixel); // top 1/2-3/4 is red
    strip.fill(WhiteColor, int(strip.numPixels() * 3/4)+g_LedProgramCurrentPixel, strip.numPixels()); // top 3/4 is white
    
    strip.show();
}

/***********************************************************************/
// Fire
// make it burn!
/***********************************************************************/
// Helper to make fire colors
void setPixelHeatColor (int Pixel, byte temperature) {
    // Scale 'heat' down from 0-255 to 0-191
    byte t192 = round((temperature/255.0)*191);
    
    // calculate ramp up from
    byte heatramp = t192 & 0x3F; // 0..63
    heatramp <<= 2; // scale up to 0..252
    
    // figure out which third of the spectrum we're in:
    if( t192 > 0x80) {                     // hottest
        setPixel(Pixel, 255, 255, heatramp);  // RGB = 255, 255, heat.... GRB = 255, 255, heat
    } else if( t192 > 0x40 ) {             // middle
        setPixel(Pixel, heatramp, 255, 0);   // RGB = 255, heat, 0.... GRB = heat, 255, 0
    } else {                               // coolest
        setPixel(Pixel, 0, heatramp, 0);     // RGB = heat, 0, 0.... GRB = 0, heat, 0
    }
}

void Fire(int Cooling, int Sparking) {
    static byte heat[NUMPIXELS];
    int cooldown;
    
    // Step 1.  Cool down every cell a little
    for( int i = 0; i < strip.numPixels(); i++) {
        cooldown = random(0, ((Cooling * 10) / strip.numPixels()) + 2);
        
        if(cooldown>heat[i]) {
            heat[i]=0;
        } else {
            heat[i]=heat[i]-cooldown;
        }
    }
    
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= strip.numPixels() - 1; k >= 2; k--) {
        heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if( random(255) < Sparking ) {
        int y = random(7);
        heat[y] = heat[y] + random(160,255);
        //heat[y] = random(160,255);
    }
    
    // Step 4.  Convert heat to LED colors
    for( int j = 0; j < strip.numPixels(); j++) {
        setPixelHeatColor(j, heat[j] );
    }
    
    strip.show();
}

/***********************************************************************/
// Running lights
// like niterider?
/***********************************************************************/
void RunningLights(byte red, byte green, byte blue) {
    // use g_LedProgramCurrentPixel to track current position
    g_LedProgramCurrentPixel++;
    for(int i=0; i < strip.numPixels(); i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+g_LedProgramCurrentPixel) * 127 + 128)/255)*red,
                 ((sin(i+g_LedProgramCurrentPixel) * 127 + 128)/255)*green,
                 ((sin(i+g_LedProgramCurrentPixel) * 127 + 128)/255)*blue);
    }
    
    strip.show();
}


/***********************************************************************/
// Sparkles Decay
// Sparkles that fade to black slowly
/***********************************************************************/
uint16_t SparkleDecay(int red, int green, int blue, int fadeDelay, int endPause) {

    // use g_LedProgramState for on/off/pause state control
    // use g_LedProgramCurrentPixel for tracking which pixels are sparkling
    static uint8_t fadeCount = 0;
    uint16_t delayForNextUpdateMs = fadeDelay;
    
    switch (g_LedProgramState) {
        default:
        case 0: // initialize and turn on
            // select a random pixel to turn on
            g_LedProgramCurrentPixel = random(strip.numPixels());
            
            // initial all the states
            fadeCount = 0;
            
            // set a random pixel and its neighbor to the desired color
            strip.setPixelColor(g_LedProgramCurrentPixel, red, green, blue);
            strip.setPixelColor(g_LedProgramCurrentPixel+1, red, green, blue);
            strip.show();
            
            g_LedProgramState = 1;
            delayForNextUpdateMs = fadeDelay;
            break;
        case 1: // off
            // increment the number of strobe flashes
            fadeCount++;
            
            // fade the pixels
            fadeToBlack(g_LedProgramCurrentPixel, fadeCount);
            fadeToBlack(g_LedProgramCurrentPixel+1, fadeCount);
            
            if(fadeCount < 100){
                // keep fading
                delayForNextUpdateMs = fadeDelay;
                g_LedProgramState = 1;
            } else {
                // done flashing, do a pause and pick a new pixel to flash
                strip.clear();
                delayForNextUpdateMs = endPause;
                g_LedProgramState = 0;
            }
            strip.show();
            break;
    }
    
    return delayForNextUpdateMs;
}

//################
//  Sparkle(255, 255, 255, 0, 10);
/***********************************************************************/
// Sparkle
// turn on several groups of pixels and then turn everything off
/***********************************************************************/
uint16_t Sparkle(byte red, byte green, byte blue, int sparksPerFlash, int sparkleDelay, int endPause) {
    
    // use g_LedProgramState for on/off state tracking
    uint16_t delayForNextUpdateMs = sparkleDelay;
    
    switch(g_LedProgramState){
        default:
            strip.clear();
            g_LedProgramState = 0;
            // fall through
        case 0: // on
            // turn on several groups of pixels
            for (int i = 0; i < sparksPerFlash; i++) {
                int Pixel = random(strip.numPixels());
                setPixel(Pixel-1,red,green,blue);
                setPixel(Pixel,red,green,blue);
                setPixel(Pixel+1,red,green,blue);
            }
            strip.show();
            delayForNextUpdateMs = sparkleDelay;
            g_LedProgramState++;
            break;
        case 1:
            strip.clear();
            strip.show();
            delayForNextUpdateMs = endPause;
            g_LedProgramState = 0;
    }
    
    return delayForNextUpdateMs;
}


/***********************************************************************/
// Read the accelerometer to monitor for motion
/***********************************************************************/
int ReadAccel() {
    lis.read();      // get X Y and Z data at once
    // Then print out the raw data
    if (testMode >= 2) {
        Serial.print("X:  "); Serial.print(abs(lis.x));
        Serial.print("  \tY:  "); Serial.print(abs(lis.y));
        Serial.print("  \tZ:  "); Serial.print(abs(lis.z));
        
        if (testMode >= 3) {
            /* Or....get a new sensor event, normalized */
            sensors_event_t event;
            lis.getEvent(&event);
            
            /* Display the results (acceleration is measured in m/s^2) */
            Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
            Serial.print(" \tY: "); Serial.print(event.acceleration.y);
            Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
            Serial.print(" m/s^2 ");
        }
        
        Serial.println();
    }
    int accelMagnitude = (abs(lis.x) + abs(lis.y) + abs(lis.z));  // normallize the movement such that sitting on a table is ~7600.
    
    return accelMagnitude;
    
}

/***********************************************************************/
/***********************************************************************/
// Main loop
// This main loop is used as a poor-man's RTOS. Since the RH_RF69 library
// uses a ton of the M0's timers (and the timer configuration seems highly
// variable from chipset to chipset), we can't easily use built in timers
// to trigger all the actions we want. Also, we don't need most of these
// actions to have high precision timing... just basically within a few
// tens of milliseconds.
// So... all functions in this loop should be minimally blocking. Anything
// expected to take more than 1-2ms should be unrolled in a manner so
// that the the system can return to the loop().
/***********************************************************************/
/***********************************************************************/
void loop() {

    static bool isLEDOn = false;
     
    // current time info
    unsigned long currentMillis = millis();
    
    // timer statics for measuring time since last action
    static unsigned long previousLedUpdateMillis = 0;
    static unsigned long previousTransmitMillis = 0;
    static unsigned long previousPriorityUpdateMillis = 0;

    // timer statics for checking Accel
    static unsigned long previousAccelCheckMillis = currentMillis;
    const unsigned long MovementThreshold = 12000; // Movement is normallized such that sitting on a table ~7600-9200
    const unsigned long AccelCheckPeriodMs = 250; // Update time between checking accel to see if we are moving
    const unsigned long notMovingTimeout = 480*1000; // how long to wait before giong to still program in ms
    static int notMovingTimer = 0; // timer for how many non-moving accelerometer measurements have been made
    const int StillProgram = 21; // pick a program to run when we are still

    // Broadcast timing
    const unsigned long transmitPeriodMs = 1000; // how long to wait between broadcasts in ms
    const int minRssiThreshold = -80;  // recieve threshold

    
    // led program controls
    const unsigned long priorityDecrementPeriodMs = 100;  // decrement the priority every X milliseconds. 100 means decrement the priority every 10ms.
    const unsigned long minimumProgramTimeMs = 1000;  // How long to run a program after switching programs
    
    const uint8_t numLedPrograms = 20; // max case id, not count
    const uint8_t defaultLedProgram = 5;
    static uint8_t overrideProgram = 0; // for testing, we want a static program
    static uint8_t currentLedProgram = defaultLedProgram;
    static uint8_t previousLedProgram = defaultLedProgram;
    static uint8_t requestedLedProgram = defaultLedProgram;
    
    static int16_t currentProgramPrioity = 50+(minimumProgramTimeMs / priorityDecrementPeriodMs);
    static int8_t requestedProgramPrioity = 0;
    
    static unsigned long ledUpdatePeriodMs = 10;  // this is delay waited before looping back through the LED case. A longer time here means the LEDs stay static with the current string display. This also blocks looking for recieved packets.
    
    /***********************************************************************/
    // Program priority update
    /***********************************************************************/

    if(currentMillis - previousPriorityUpdateMillis >= priorityDecrementPeriodMs){
        // Update the current priority based on how long it's been runnging.
        // If the priority is 0, just keep it 0
        currentProgramPrioity = currentProgramPrioity == 0 ? 0 : (currentProgramPrioity - ((currentMillis - previousPriorityUpdateMillis)/priorityDecrementPeriodMs) );
        // If the priority went negative, fix it to 0
        currentProgramPrioity = currentProgramPrioity < 0 ? 0 : currentProgramPrioity;
        // Update priority update timestamp
        previousPriorityUpdateMillis = currentMillis;
        
        // Create a heartbeat LED flash to show we're updating the priority
        digitalWrite(LED_PIN, isLEDOn);
        isLEDOn = !isLEDOn;
        
        char buffer[255];
        sprintf(buffer, "%ld: prg: %d pri: %d updateWait: %d ms still for: %d", currentMillis, currentLedProgram, currentProgramPrioity, ledUpdatePeriodMs, notMovingTimer);
        Serial.println((char*)buffer);
        
    }
    /***********************************************************************/
    // check Accel to see if we are moving
    /***********************************************************************/
    if ((currentMillis - previousAccelCheckMillis >= AccelCheckPeriodMs) && (useAccel >= 1)){
        // update accel check timestamp
        previousAccelCheckMillis = currentMillis;
        
        // get the current accel data
        int accelMagnitude = ReadAccel();
        
        // If the accel data is valid and we have significant movement, time how long we haven't been moving
        if ((accelMagnitude < MovementThreshold) && (accelMagnitude > 0)) {
            notMovingTimer = (notMovingTimer + (currentMillis - previousAccelCheckMillis));
        } else {
            notMovingTimer = 0;  // reset the counter since we are moving again
            overrideProgram = 0;
        }
        
        // If we haven't been moving for a long time, override the program
        if (notMovingTimer > notMovingTimeout) {
            overrideProgram = StillProgram; // go to a low power sparkly program
            char buffer[255];
            sprintf(buffer, "%ld: Not Moving! Overriding to prg: %d pri: %d STILL FOR: %d ms", currentMillis, overrideProgram, currentProgramPrioity, notMovingTimer);
            Serial.println((char*)buffer);
        }
    }
    
    /***********************************************************************/
    // Determine what program to run
    // We can do this as often as we like (no timer limitation)
    // Since it will only come into effect when there is an override or
    // there is a new packet recv'd
    /***********************************************************************/
    /***********************************************************************/
    // Program change
    // if there is a new program request, change to it if it has a higher
    // priority than our current priority
    // if the program changes, we want to run each program for at least 2s,
    // so we set the new priority to the requested priorty plus 2 *priorityDecrementPeriodMs
    /***********************************************************************/
    // check to see if there is a new program requested
    // and that the request has a higher priority than our current priority
    if(requestedProgramPrioity > currentProgramPrioity){
        // change the led program
        currentLedProgram = requestedLedProgram;
        previousLedProgram = currentLedProgram;
        // set the priority so it runs at least as long as our minimum
        currentProgramPrioity = requestedProgramPrioity + (minimumProgramTimeMs / priorityDecrementPeriodMs);
        
        // reset the requested info
        requestedProgramPrioity = 0;
        requestedLedProgram = 0;
        
        char buffer[255];
        sprintf(buffer, "%ld: Changing Higher Priorty Prg prg: %d pri: %d", currentMillis, currentLedProgram, currentProgramPrioity);
        Serial.println((char*)buffer);
        
        // give the new program a blank slate to play with
        resetAllLedProgramStates();
    }
    
    // if there is an override program number use that program.
    if (overrideProgram != 0) {
        // if this is the first time here, save the program history so we can switch back
        if(currentLedProgram != overrideProgram) {
            previousLedProgram = currentLedProgram;
            char buffer[255];
            sprintf(buffer,"%d: Overiding Program! prg: %d", currentMillis, overrideProgram);
            Serial.println((char*)buffer);
        }
        
        // override the current program
        currentLedProgram = overrideProgram;
    }
    else {
        // if we're not overriding the program anymore, restore the previous program
        // since we might have recieved a new program request since we were last moving
        // the new program might be different than it was when we went to override
        currentLedProgram = previousLedProgram;
    }
    
    /***********************************************************************/
    // Led update
    // update the LEDs based on the current program
    /***********************************************************************/
    if (currentMillis - previousLedUpdateMillis >= ledUpdatePeriodMs){
        // update the previous time record
        previousLedUpdateMillis = currentMillis;
        
        // play the current LED program
        switch(currentLedProgram){
            default:
                Serial.println("unknown LED program");
                // fall through to use 0 as default
            case 0: // red color wipe  variables: byte red, byte green, byte blue, wait before adding each LED
                ledUpdatePeriodMs = 3;
                colorWipe (0, 150, 0);
                break;
            case 1: // green color wipe
                ledUpdatePeriodMs = 3;
                colorWipe (150, 0, 0);
                break;
            case 2: // blue color wipe
                ledUpdatePeriodMs = 3;
                colorWipe (0, 0, 150);
                break;
            case 3: // purple color wipe
                ledUpdatePeriodMs = 3;
                colorWipe (0, 75, 75);
                break;
            case 4: // rainbow
                ledUpdatePeriodMs = 10;
                rainbow(); // rainbow
                break;
            case 5: // rainbowCycle
                ledUpdatePeriodMs = 10;
                rainbowCycle();
                break;
            case 6: // blue color chase
                ledUpdatePeriodMs = 50;
                theaterChase(strip.Color(0, 0, 255));
                break;
            case 7: // red color chase
                ledUpdatePeriodMs = 50;
                theaterChase(strip.Color(0, 255, 0));
                break;
            case 8: // green color chase
                ledUpdatePeriodMs = 50;
                theaterChase(strip.Color(255, 0, 0));
                break;
            case 9: // purple color chase
                ledUpdatePeriodMs = 50;
                theaterChase(strip.Color(0, 75, 75));
                break;
            case 10: // color chase
                ledUpdatePeriodMs = 50;
                theaterChaseRainbow();
                break;
            case 11: // poice mode
                // variable update rate based on state of the program
                ledUpdatePeriodMs = policeMode();
                break;
            case 12: // china poice mode
                // variable update rate based on state of the program
                ledUpdatePeriodMs = policeChinaMode2(10,20,300, false); // china Police Mode variable: wait between switching colors
                break;
            case 13: // china poice mode Half of the strip
                // int StrobeCount, int FlashDelay, int EndPause, bool halfString
                ledUpdatePeriodMs = policeChinaMode2 (10, 20, 300, true); // china Police Mode Half and Half variable:
                break;
            case 14: // color wipe random color and back to black
                ledUpdatePeriodMs = 3;
                colorWipe ((random(0,200)),(random(0,100)),(random(0,200)));
                break;
            case 15: // meteor variables: red,  green,  blue,  meteorSize,  meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay
                ledUpdatePeriodMs = 10;
                meteorRain(100,255,200, 10, 50, true);
                break;
            case 16:  // Strobe!
                // variable update rate based on state of the program
                // params: byte red, byte green, byte blue, int strobeCount, int flashDelay, int endPause
                ledUpdatePeriodMs = Strobe(150,100,200, 10, 25, 500);
                break;
            case 17: // Fire! variables: int Cooling, int Sparking, int SpeedDelay
                ledUpdatePeriodMs = 15;
                Fire(55,120);
                break;
            case 18:  // Running lights variables: byte red, byte green, byte blue, int WaveDelay
                ledUpdatePeriodMs = 10;
                RunningLights(0,150,150);
                break;
            case 19: // Sparkle
                // variable update rate based on state of the program
                ledUpdatePeriodMs = Sparkle(255, 255, 255, 3, 5, 5);
                break;
            case 20: // Sutro
                ledUpdatePeriodMs = 20;
                Sutro();
                break;
            // Sparkle slow is used for when there is no motion
            case 21: // Sparkle slow
                // variable update rate based on state of the program
                ledUpdatePeriodMs = Sparkle(150, 150, 150, 2, 10, 200);
                break;
            case 22: // SparkleDecay
                // variable update rate based on state of the program
                ledUpdatePeriodMs = SparkleDecay((random(0,200)),(random(0,100)),(random(0,200)), 5, 0);
                break;
        }
    }
    
    /***********************************************************************/
    // Packet receive
    // Check if there is a new packet available, if there is, and the rssi
    // is above a threshold, decode it and
    // use it as the requested program and priority
    /***********************************************************************/
    if (testMode == 10) {  // etra logging to see how often we are checking for packets
        char buffer[255];
        sprintf(buffer, "%ld: Looking for Received Packets", currentMillis);
        Serial.println(buffer);
    }

    if (rf69.available()){
        char buffer[255];
        uint8_t packet[RH_RF69_MAX_MESSAGE_LEN];
        int lastRssi = -100;
        uint8_t len=255;

        
        // read in the packet
        if (rf69.recv((uint8_t*)packet, &len) && len > 0) {
            //null terminate just in case since we treat this like a char*
            packet[len] ='\0';
            lastRssi = rf69.lastRssi();
            
//            sprintf(buffer, "%ld: Received Packet Rssi: %d len: %d \"%s\" ", currentMillis, lastRssi, len, (char*)packet );
//            Serial.println(buffer);

            char *data = (char*)packet;
              
            // check the packet's rssi
            if(lastRssi > minRssiThreshold){
                int tempProgram;
                int tempPriority;
                int numFound = 0;
                
                numFound = sscanf(data, "%d %d", &tempProgram, &tempPriority);
                
                // if we got two items parsed out of the packet, use them for req
                if (numFound == 2){
                    requestedProgramPrioity =  tempPriority;
                    requestedLedProgram =  tempProgram;

                    sprintf(buffer, "%ld: Got request: %d %d", currentMillis, requestedLedProgram, requestedProgramPrioity);
                    Serial.println(buffer);

                }
                else {
                    sprintf(buffer, "Bad packet. Found %d items. %s", numFound, data);
                    Serial.println(buffer);
                }
            } // end if rssi threshold
        } // end if recv packet
        else {
            char buffer[255];
            sprintf(buffer, "%ld: Packet receive failed. len: %d", currentMillis, len);
            Serial.println(buffer);
            lastRssi = -100;
        }  // end else recv packet
    } // end rf69 packet available
    
    /***********************************************************************/
    // Packet Transmission
    // Send out a new program choice by generating a random program numer
    // and a random program priority
    // if this new priority is higher than the current priority, switch our
    // current program
    /***********************************************************************/
      if (currentMillis - previousTransmitMillis >= transmitPeriodMs){
            char buffer[255];

            if (transmitMode == 1) {

                char radiopacket[RH_RF69_MAX_MESSAGE_LEN];

                // generate a random new program with random priority
                int tempPriority = (int16_t)random(1, minimumProgramTimeMs / priorityDecrementPeriodMs);
                int tempProgram = (uint8_t)random(0, numLedPrograms +1); //min inclusive, max exclusive
                sprintf(buffer, "%ld: RANDOM prg:%d pri:%d   Requested prg:%d pri:%d", currentMillis, tempProgram, tempPriority, requestedLedProgram, requestedProgramPrioity);
                Serial.println(buffer);

                sprintf(buffer, "%ld: Sending RANDOM prg:%d pri:%d Previous Transmit: %d ms (%d late)", currentMillis, tempProgram, tempPriority, (currentMillis - previousTransmitMillis), ((currentMillis - previousTransmitMillis) - transmitPeriodMs ));
                Serial.println(buffer);
//              int randomWait = random(0,10);
//              delay(randomWait);  // prevents everyone from transmitting at the same time.
//              sprintf(radiopacket, "%d %d", requestedLedProgram, requestedProgramPrioity);
                // Send a message!
                rf69.send((uint8_t*)radiopacket, strlen(radiopacket));
                rf69.waitPacketSent();

                requestedProgramPrioity =  tempPriority;
                requestedLedProgram =  tempProgram;                                    
               
            } else {
                  // Log that we we aren't transmitting. Normally used when testing just one unit.
                  sprintf(buffer, "%ld: Not transmitting because transmitMode != 1, Previous Transmit: %d ms (%d late)", currentMillis, (currentMillis - previousTransmitMillis), ((currentMillis - previousTransmitMillis) - transmitPeriodMs ));
                  Serial.println(buffer);
            }

            if(testMode == 1 ) { 
                  requestedLedProgram = currentLedProgram+1; 
                  if (requestedLedProgram >= numLedPrograms) {requestedLedProgram = 0;}
                  
                  requestedProgramPrioity =  (minimumProgramTimeMs / priorityDecrementPeriodMs);
                  sprintf(buffer, "%ld: Test Mode Requesting Next Prg prg:%d pri:%d ", currentMillis, requestedLedProgram, requestedProgramPrioity);
                  Serial.println(buffer);
            }
            
        previousTransmitMillis = currentMillis;  // update time since last transmitted to now
    }

    
}
