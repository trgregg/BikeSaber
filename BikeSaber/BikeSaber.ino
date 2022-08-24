///
/// @mainpage	blinkLed
///
/// @file    BikeSaber.ino
/// @brief    Main sketch for Burning Man BikeSabers
/// @details	<#details#>
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Justin Gregg
/// @author		Rodentia
/// @author   Travis Gregg
/// @author   Marroug

/// @date		7/18/18 5:45 PM
/// @version	<#version#>
///
/// @copyright	(c) Justin Gregg, 2018
/// @copyright  (c) Travis Gregg, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
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

// Accelerometer 
#include "Adafruit_LIS3DH.h"

// Accelerometer via I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

// Time of Flight Sensor
//#include "Adafruit_VL6180X.h"
//Adafruit_VL6180X vl = Adafruit_VL6180X();
#include <Adafruit_VL53L0X.h>
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Define structures and classes


// BIKE LIGHTS
// Define variables and constants
const int lessLight = 0;  // use this for longer strings. It will add this number to the LED to skip to limit power.
const int testMode = 0;     // If testing with just one BikeSaber, use this mode which: moves to the next program sequentially
static int transmitMode = 1;  // use this for BikeSabers that we only want to recieve, but not vote.
static int useAccel = 1; // we will set this to 0 if we can't find accel
static int useToF = 0; // we will set this to 0 if we can't find Time of Flight sensor 
static int useAnalog = 0; // we will set this to 0 if we don't want to look at the analog input for overrides

//#define NUMPIXELS 100  // For Spiral Bike Whips
//const int ledUpdateScaler = 9; // For Bike Whips

#define NUMPIXELS 90  // For Neon Bike Whips
const int ledUpdateScaler = 11; // For Neon Bike Whips

//#define NUMPIXELS 55  // For Kids Bike Whips
//const int ledUpdateScaler = 16; 


// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Setup
// ***************************************************************************
// Stuff for LED string
// ***************************************************************************
//#define NUMPIXELS 130  // For Bike Whips
//#define NUMPIXELS 300  // For full strips
//#define NUMPIXELS 50 // For Bike Wheels
//#define NUMPIXELS 900  // For Frence
//
//#define PIXEL_PIN 6
//#include <Adafruit_NeoPixel.h>
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800); // for 8mm NeoPixels

//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

//#define PIXEL_PIN 11
#define PIXEL_PIN 5
#include <Adafruit_NeoPixel_ZeroDMA.h>
Adafruit_NeoPixel_ZeroDMA strip = Adafruit_NeoPixel_ZeroDMA(NUMPIXELS, PIXEL_PIN, NEO_RGB);


// ***************************************************************************
// Stuff for RFM69
// ***************************************************************************
// Add setup code
#define RF69_FREQ 915.0

//#if defined (ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define ANALOG1       A1
//#define LED           13
//#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

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
     if (testMode >= 2) {while (!Serial);}     // will pause Zero, Leonardo, etc until serial console opens

    pinMode(LED_PIN, OUTPUT);
    randomSeed(millis()%255);

/////// Setup Radio RFM69
    //pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
    Serial.println("Feather RFM69 TX Test!");
    
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
    
    //pinMode(LED, OUTPUT);
    
    Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");


///////// setup accelerometer stuff
    Serial.println("LIS3DH test");
    
    if (useAccel ==1) { 
      if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
          Serial.println("Couldnt start accel. Continuing without it.");
          useAccel = 0;
        
        } else {
          Serial.println("LIS3DH found!");
          
          lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
          
          Serial.print("Range = "); Serial.print(2 << lis.getRange());  
          Serial.println("G");
        }
    }

///////// setup Time of Flight stuff
    Serial.println("Time of Flight VL53L0X test");
    if ( useToF == 1) {
        if (! lox.begin(0x29)) {
          Serial.println("Failed to find ToF. Continuing without it.");
          useToF = 0;
        
        } else {
        Serial.println("VL53L0X ToF found!");
        }
    }
    
////// Setup the NeoPixel string
    Serial.println("Adafruit NeoPixel initializing...");
    strip.begin(); // This initializes the NeoPixel library.
    strip.show(); // start with everything off
    Serial.println("Adafruit NeoPixel  initialized!");
    
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
    strip.setBrightness(255);  // reset overall strip brightness
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
	if ( lessLight > 0 ) { g_LedProgramCurrentPixel= g_LedProgramCurrentPixel + lessLight;} // Skip pixels if we want less light
    g_LedProgramCurrentPixel++;
    
    // if we've filled the strip, flip on/off and refill
    // add a few pixels over the strip length to make it stay lit/off for a bit
    if(g_LedProgramCurrentPixel >= ( strip.numPixels() + (strip.numPixels() * 0.25) ) ) {  // over fill by 25% so we stay full on/off for some time.
        g_LedProgramState++;
        g_LedProgramCurrentPixel = 0;
    }
}

void colorWipeAndFade(int red, int green, int blue) {
     // reduce red a percentage of the number of pixels on each pixel so we reach 0 at the end
     int redFade = red - int(red * g_LedProgramCurrentPixel / strip.numPixels() );
     int greenFade = green - int(green * g_LedProgramCurrentPixel / strip.numPixels() );
     int blueFade = blue - int(blue * g_LedProgramCurrentPixel / strip.numPixels() );

    // use g_LedProgramState for wipe on/off state
    switch(g_LedProgramState){
        default:
            g_LedProgramState = 0;
            g_LedProgramCurrentPixel = 0;
            // fall through
        case 0: // wipe color on
            strip.setPixelColor(g_LedProgramCurrentPixel, strip.Color(greenFade, redFade, blueFade));
            strip.show();
            break;
        case 1: // wipe color off
            strip.setPixelColor(g_LedProgramCurrentPixel, 0);
            strip.show();
            break;
    }
    
    // Skip pixels if we are using long strips and want to reduce power
    if ( lessLight > 0 ) { g_LedProgramCurrentPixel= g_LedProgramCurrentPixel + lessLight;} // Skip pixels if we want less light

    // move to the next pixel
    g_LedProgramCurrentPixel++;
    
    // if we've filled the strip, flip on/off and refill
    // add a few pixels over the strip length to make it stay lit/off for a bit
    if(g_LedProgramCurrentPixel >= ( strip.numPixels() + (strip.numPixels() * 0.25) ) ) {  // over fill by 25% so we stay full on/off for some time.
        g_LedProgramState++;
        g_LedProgramCurrentPixel = 0;
    }
}


void colorWipeRainbow( int32_t colorSeed ) {
    // use g_LedProgramState for wipe on/off state
    switch(g_LedProgramState){
        default:
            g_LedProgramState = 0;
            g_LedProgramCurrentPixel = 0;
            // fall through
        case 0: // wipe color on
            g_LedProgramColor = Wheel((colorSeed/5) & 255); // change color on every pixel using the current priority (counting down) to determine the color
            strip.setPixelColor(g_LedProgramCurrentPixel, g_LedProgramColor);
            strip.show();
            break;
        case 1: // wipe color off
            strip.setPixelColor(g_LedProgramCurrentPixel, 0);
            strip.show();
            break;
    }
    
    // move to the next pixel
    if ( lessLight > 0 ) { g_LedProgramCurrentPixel= g_LedProgramCurrentPixel + lessLight;} // Skip pixels if we want less light
    g_LedProgramCurrentPixel++;
    
    // if we've filled the strip, flip on/off and refill
    // add a few pixels over the strip length to make it stay lit/off for a bit
    if(g_LedProgramCurrentPixel >= strip.numPixels()+25){
        g_LedProgramState++;
        g_LedProgramCurrentPixel = 0;
    }
}

void colorWipeRandom( int32_t colorSeed ) {
    // use g_LedProgramState for wipe on/off state
    switch(g_LedProgramState){
        default:
            g_LedProgramState = 0;
            g_LedProgramCurrentPixel = 0;
            g_LedProgramColor = Wheel((colorSeed/5) & 255);  // pick a color using the current priority (counting down) and use it to wipe the stip
//            strip.setBrightness(175);  // set the brightness a little lower since some colors can draw too much power
            // fall through
        case 0: // wipe color on
            strip.setPixelColor(g_LedProgramCurrentPixel, g_LedProgramColor);
            strip.show();
            break;
        case 1: // wipe color off
            strip.setPixelColor(g_LedProgramCurrentPixel, 0);
            strip.show();
            break;
    }
    
    // move to the next pixel
    if ( lessLight > 0 ) { g_LedProgramCurrentPixel= g_LedProgramCurrentPixel + lessLight;} // Skip pixels if we want less light
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
uint32_t policeMode() {
    uint32_t delayForNextUpdateMs = 100;
    switch(g_LedProgramState){
        default:
        case 0: // red color wipe
            g_LedProgramState++;
            for(int i=0; i < (strip.numPixels()); i++){
                if ( lessLight > 0 ) { i=i+lessLight;}   //turn off every other pixel
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
                if ( lessLight > 0 ) { i=i+lessLight;}   //turn off every other pixel
                strip.setPixelColor(i, strip.Color(0, 0, 255));
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
uint32_t policeChinaMode2(int strobeCount, int flashDelay, int endPause, bool halfString) {
    // use g_LedProgramCurrentPixel as a strobe counter
    // use g_LedProgramState for state machine state
    // use g_LedProgramColor for current color
    uint32_t delayForNextUpdateMs = 10;
    const uint32_t colorsForFlashing[2] = {strip.Color(0, 200, 0), strip.Color(0, 0, 255)};
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
                    for(int i=0; i < (NUMPIXELS/2); i++) {strip.setPixelColor(i, g_LedProgramColor); i=i+lessLight/2;} 

//                    strip.fill(g_LedProgramColor, 0, (NUMPIXELS/2));
                } else {
                    for(int i=NUMPIXELS/2; i < NUMPIXELS; i++) {strip.setPixelColor(i, g_LedProgramColor); i=i+lessLight/2;} 

//                    strip.fill(g_LedProgramColor, (NUMPIXELS/2), NUMPIXELS);
                }
            }
            else {
                // for full string, just fill the full string with the current color
//                strip.fill(g_LedProgramColor,0, NUMPIXELS);
                for(int i=0; i < (strip.numPixels()); i++) {strip.setPixelColor(i, g_LedProgramColor); i=i+lessLight;} 

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
                    g_LedProgramColor = colorsForFlashing[0]; // dim red
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
        if ( lessLight +1 > 0 ) { strip.setPixelColor(i, 0); i=i+lessLight+1;}   //turn off every other pixel
        strip.setPixelColor(i, Wheel((i+g_LedProgramColor) & 255));
    }
    
    // show the rainbow
    strip.setBrightness(200); // dim down a little
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
        if ( lessLight > 0 ) { strip.setPixelColor(i, 0); i=i+lessLight;}   //turn off every other pixel
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
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3+lessLight) {
        //turn off every third pixel from the previous run
        strip.setPixelColor(i+g_LedProgramCurrentPixel-1, 0);
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3+lessLight) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel, color);    //set every third pixel 
    }
    strip.show();

    // move the starting pixel forward one
    g_LedProgramCurrentPixel++;
    
    // reset to first pixel after moving 3
    if(g_LedProgramCurrentPixel >= 3+lessLight) g_LedProgramCurrentPixel = 0;

}

/***********************************************************************/
// Theater chase random color
// Theatre-style crawling lights.
/***********************************************************************/
void theaterChaseRandom(uint32_t color) {
    // use g_LedProgramCurrentPixel for current pixel position
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3+lessLight) {
        //turn off every third pixel from the previous run
        strip.setPixelColor(i+g_LedProgramCurrentPixel-1, 0);
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3+lessLight) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel, (Wheel((color/10) & 255)));    //set every third pixel 
    }
    strip.show();

    // move the starting pixel forward one
    g_LedProgramCurrentPixel++;
    
    // reset to first pixel after moving 3
    if(g_LedProgramCurrentPixel >= 3+lessLight) g_LedProgramCurrentPixel = 0;

}

/***********************************************************************/
//Theatre-style crawling lights with rainbow effect
/***********************************************************************/
void theaterChaseRainbow() {
    // use g_LedProgramCurrentPixel for current pixel position
    // use g_LedProgramColor for rainbox color position
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3+lessLight) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel-1, 0);        //turn every third pixel off
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3+lessLight) {
        strip.setPixelColor(i+g_LedProgramCurrentPixel, Wheel( (i+g_LedProgramColor) % 255));    //turn every third pixel on
    }
 
    strip.show();

    // move the starting pixel forward one
    g_LedProgramCurrentPixel++;
    // reset to first pixel after moving 3
    if(g_LedProgramCurrentPixel >= 3+lessLight) g_LedProgramCurrentPixel = 0;
    
    // increment the initial color position
    g_LedProgramColor++;
    if(g_LedProgramColor > 255) g_LedProgramColor = 0;
}


///***********************************************************************/
// Meteor rainBow
// make it rain glowing rocks
/***********************************************************************/
void meteorRainbow(int32_t colorSeed, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay) {
    // use g_LedProgramCurrentPixel for meteor position
    // use g_LedProgramState for initialization flag
    
    // initialize or reset the initial meteor position
    if(g_LedProgramState == 0){
        g_LedProgramCurrentPixel = strip.numPixels() + (strip.numPixels()/8);
        g_LedProgramState++;
        g_LedProgramColor = Wheel((colorSeed) & 255); // change color on every pixel using the current priority (counting down) to determine the color
    }
    
    // reset the meteor position after it shoots through
    if(g_LedProgramCurrentPixel <= (0 - meteorSize - meteorTrailDecay)){
        g_LedProgramCurrentPixel = strip.numPixels() + (strip.numPixels()/8);
        g_LedProgramColor = Wheel((colorSeed) & 255); // change color on every pixel using the current priority (counting down) to determine the color
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
            strip.setPixelColor(g_LedProgramCurrentPixel - j, g_LedProgramColor);
        }
    }
    
    strip.show();
    g_LedProgramCurrentPixel--;

}


/***********************************************************************/
// Strobe
// Flashy all the string with one color
/***********************************************************************/
uint32_t Strobe(byte red, byte green, byte blue, int strobeCount, int flashDelay, int endPause){
    // use g_LedProgramState for on/off/pause state
    // use g_LedProgramCurrentPixel as a strobe counter
    
    uint32_t strobeColor = strip.Color(red, green, blue);
    uint32_t delayForNextUpdateMs = flashDelay;
    
    switch(g_LedProgramState){
        default:
        case 0: // initialize
            g_LedProgramCurrentPixel = 0;
            g_LedProgramState++;
            // fall through
        case 1: // turn all on
            // strip.fill(strobeColor, 0, strip.numPixels());
			for(int i=0; i < (strip.numPixels()); i++) {strip.setPixelColor(i, strobeColor); i=i+lessLight;} 
			
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
// Strobe Random Rainbow color
// Flashy all the string with one color
/***********************************************************************/
uint32_t StrobeRainbow(int32_t colorSeed, int strobeCount, int flashDelay, int endPause){
    // use g_LedProgramState for on/off/pause state
    // use g_LedProgramCurrentPixel as a strobe counter
    
    uint32_t strobeColor = g_LedProgramColor;
    uint32_t delayForNextUpdateMs = flashDelay;
    
    switch(g_LedProgramState){
        default:
        case 0: // initialize
            g_LedProgramColor = Wheel((colorSeed/5) & 255); // change color on every pixel using the current priority (counting down) to determine the color
            g_LedProgramCurrentPixel = 0;
            g_LedProgramState++;
            // fall through
        case 1: // turn all on
            // strip.fill(strobeColor, 0, strip.numPixels());
            for(int i=0; i < (strip.numPixels()); i++) {strip.setPixelColor(i, strobeColor); i=i+lessLight;} 
      
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
                g_LedProgramState = 0;
                
                // set up the delay for going to the next color
                delayForNextUpdateMs = endPause;
            }
            break;
    }
    
    return delayForNextUpdateMs;
}


/***********************************************************************/
// Sutro emulator
/***********************************************************************/
void Sutro(){
    // use g_LedProgramCurrentPixel for current position tracking
    
    const uint32_t RedColor = strip.Color(0, 150, 0);
    const uint32_t WhiteColor = strip.Color(100, 75, 150);
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
    if( lessLight > 0) {
        for(int i=0; i< strip.numPixels(); i++) {
            if ( i % (lessLight+1) != 0) {  
          strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
    }
    }
    strip.show();
}

/***********************************************************************/
// Ukraine
/***********************************************************************/
void Ukraine(){
    // use g_LedProgramCurrentPixel for current position tracking
    
    const uint32_t BlueColor = strip.Color(0, 0, 200);
    const uint32_t YellowColor = strip.Color(75, 100, 0);
    g_LedProgramCurrentPixel++;
    
    if (g_LedProgramCurrentPixel > strip.numPixels()) g_LedProgramCurrentPixel = 1;
    
    strip.fill(YellowColor, 0+g_LedProgramCurrentPixel-int(strip.numPixels() *3/4),
               0+g_LedProgramCurrentPixel-int(strip.numPixels()/2));
    strip.fill(BlueColor, 0+g_LedProgramCurrentPixel-int(strip.numPixels()/2),
               0+g_LedProgramCurrentPixel-int(strip.numPixels()/4));
    strip.fill(YellowColor, 0+g_LedProgramCurrentPixel-int(strip.numPixels()/4),
               0+g_LedProgramCurrentPixel);
    strip.fill(BlueColor, 0+g_LedProgramCurrentPixel,
               int(strip.numPixels()/4)+g_LedProgramCurrentPixel); // bottom 1/4 is red
    strip.fill(YellowColor, int(strip.numPixels()/4)+g_LedProgramCurrentPixel,
               int(strip.numPixels()/2)+g_LedProgramCurrentPixel); // bottom 1/4-1/2 is white
    strip.fill(BlueColor, int(strip.numPixels()/2)+g_LedProgramCurrentPixel,
               int(strip.numPixels() * 3/4)+g_LedProgramCurrentPixel); // top 1/2-3/4 is red
    strip.fill(YellowColor, int(strip.numPixels() * 3/4)+g_LedProgramCurrentPixel, strip.numPixels()); // top 3/4 is white
    if( lessLight > 0) {
        for(int i=0; i< strip.numPixels(); i++) {
            if ( i % (lessLight+1) != 0) {  
          strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
    }
    }
    strip.show();
}



/***********************************************************************/
// Fire from the first pixel
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
    for( int i = 0 ; i < strip.numPixels(); i++) {
//        cooldown = (int)random(0, ((Cooling * 10) / strip.numPixels()) + 2);
        cooldown = (int)random(0, ((Cooling * 10) / strip.numPixels()) + .222 * ledUpdateScaler );        
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
        int y = (int)random(7);
        heat[y] = heat[y] + random(160,255);
        //heat[y] = random(160,255);
    }
    
    // Step 4.  Convert heat to LED colors
    for( int j = 0; j < strip.numPixels(); j++) {
        j= j+lessLight;
        setPixelHeatColor(j, heat[j] );
    }
    
    strip.show();
}

/***********************************************************************/
// Fire from the last pixel
// make it burn!
/***********************************************************************/
void FireUp(int Cooling, int Sparking) {
    static byte heat[NUMPIXELS];
    int cooldown;
    
    // Step 1.  Cool down every cell a little
    for( int i = strip.numPixels(); i > 0; i--) {
//        cooldown = (int)random(0, ((Cooling * 10) / strip.numPixels()) + 2);
        cooldown = (int)random(0, ((Cooling * 10) / strip.numPixels()) + .222 * ledUpdateScaler );        
        if(cooldown>heat[i]) {
            heat[i]=0;
        } else {
            heat[i]=heat[i]-cooldown;
        }
    }
    
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= 0; k <= strip.numPixels() ; k++) {
        heat[k] = (heat[k + 1] + heat[k + 2] + heat[k + 2]) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if( random(255) < Sparking ) {
        int y = (int)random(7);
        heat[y] = heat[y] + random(0,95);
        //heat[y] = random(160,255);
    }
    
    // Step 4.  Convert heat to LED colors
    for( int j = strip.numPixels(); j > 0; j--) {
        j= j+lessLight;
        setPixelHeatColor(j, heat[j] );
    }
    
    strip.show();
}

/***********************************************************************/
// Running lights: Sine wave moving lights with static color
/***********************************************************************/
void RunningLights(byte red, byte green, byte blue) {
    // use g_LedProgramCurrentPixel to track current position
    g_LedProgramCurrentPixel++;
    for(int i=0; i < strip.numPixels(); i++) {
        i=i+lessLight;
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
// RainbowFill: Fill the entire strip with one color, rotate
/***********************************************************************/
void RainbowFill(int32_t colorSeed) {
    uint32_t stripColor = g_LedProgramColor;
    g_LedProgramColor = Wheel((colorSeed/5) & 255); // change color on every pixel using the current priority (counting down) to determine the color
    strip.fill(g_LedProgramColor, 0, NUMPIXELS);
    strip.show();
}

/***********************************************************************/
// RainbowStick: Rainbow the strip with one color, move 
/***********************************************************************/
void RainbowStick(int32_t colorSeed) {
    g_LedProgramCurrentPixel++;
    g_LedProgramColor = Wheel((colorSeed/5) & 255); // change color on every pixel using the current priority (counting down) to determine the color
//  strip.rainbow(first_hue, reps, saturation, brightness, bool gammify);
    strip.rainbow(g_LedProgramColor, 3, 255, 255, true);
    strip.show();

}


/***********************************************************************/
// Sparkles Decay
// Sparkles that fade to black slowly
/***********************************************************************/
uint32_t SparkleDecay(int red, int green, int blue, int fadeDelay, int endPause) {

    // use g_LedProgramState for on/off/pause state control
    // use g_LedProgramCurrentPixel for tracking which pixels are sparkling
    static uint8_t fadeCount = 0;
    uint32_t delayForNextUpdateMs = fadeDelay;
    
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
uint32_t Sparkle(byte red, byte green, byte blue, int sparksPerFlash, int sparkleDelay, int endPause) {
    
    // use g_LedProgramState for on/off state tracking
    uint32_t delayForNextUpdateMs = sparkleDelay;
    const uint32_t sparkleColor = strip.Color(red, green, blue);
    
    switch(g_LedProgramState){
        default:
            strip.clear();
            g_LedProgramState = 0;
            // fall through
        case 0: // on
            // turn on several groups of pixels
            for (int i = 0; i < sparksPerFlash; i++) {  // Draw the number of sparkles we want
                int Pixel = (int)random(strip.numPixels());   // Pick a random place on the string to start the sparkle
                for (int j = 0; j < (strip.numPixels() *.02); j++ ) {  // Fill in about 2% of the string for each sparkle
                    setPixel(Pixel-j,red,green,blue);
                }
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

//  SparkleRainbow(colorSeed, 0, 10);
/***********************************************************************/
// Sparkle
// turn on several groups of pixels and then turn everything off
/***********************************************************************/
uint32_t SparkleRainbow(int32_t colorSeed, int sparksPerFlash, int sparkleDelay, int endPause) {
    
    // use g_LedProgramState for on/off state tracking
    uint32_t delayForNextUpdateMs = sparkleDelay;
//    const uint32_t sparkleColor = strip.Color(red, green, blue);
    g_LedProgramColor = Wheel((colorSeed/5) & 255); // change color on every pixel using the current priority (counting down) to determine the color
    switch(g_LedProgramState){
        default:
            strip.clear();
            g_LedProgramState = 0;
            
            // fall through
        case 0: // on
            // turn on several groups of pixels
            for (int i = 0; i < sparksPerFlash; i++) {  // Draw the number of sparkles we want
                int Pixel = (int)random(strip.numPixels());   // Pick a random place on the string to start the sparkle
                for (int j = 0; j < (strip.numPixels() *.02); j++ ) {  // Fill in about 2% of the string for each sparkle
                    strip.setPixelColor(Pixel-j,g_LedProgramColor);
                }
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


//################
//  SparkleFlag(255, 255, 255, 0, 10);
/***********************************************************************/
// SparkleFlag
// turn on several groups of pixels and then turn everything off
/***********************************************************************/
uint32_t SparkleFlag(byte red, byte green, byte blue, int sparksPerFlash, int sparkleDelay, int endPause, int32_t colorSeed) {
    
    // use g_LedProgramState for on/off state tracking
    uint32_t delayForNextUpdateMs = sparkleDelay;
    const uint32_t sparkleColor = strip.Color(red, green, blue);
    g_LedProgramColor = Wheel((colorSeed/5) & 255);
        
    switch(g_LedProgramState){
        default:
            strip.clear();
            g_LedProgramState = 0;
            // fall through
        case 0: // on
            // turn on several groups of pixels
            for (int i = 0; i < sparksPerFlash; i++) {  // Draw the number of sparkles we want
                int Pixel = (int)random(strip.numPixels());   // Pick a random place on the string to start the sparkle
                for (int j = 0; j < (strip.numPixels() *.02); j++ ) {  // Fill in about 2% of the string for each sparkle
                    setPixel(Pixel-j,red,green,blue);
                }
            }

            for (int j = 0; j < (strip.numPixels() *.15); j++ ) {  // Fill in bottom 15% of the string for a brake light
              setPixel(j,0,50,0);
            }
              
            strip.fill(g_LedProgramColor, (strip.numPixels() *.9), strip.numPixels() ); // Fill in top 10% of the string for a flag
        
            strip.show();
            delayForNextUpdateMs = sparkleDelay;
            g_LedProgramState++;
            break;
        case 1:
            strip.clear();
            for (int j = 0; j < (strip.numPixels() *.15); j++ ) {  // Fill in bottom 15% of the string for a brake light
              setPixel(j,0,50,0);
            }
                        
            strip.fill(g_LedProgramColor, (strip.numPixels() *.9), strip.numPixels() ); // Fill in top 10% of the string for a flag

            strip.show();
            delayForNextUpdateMs = endPause;
            g_LedProgramState = 0;
    }

    return delayForNextUpdateMs;
}


/***********************************************************************/
// ToF Wipe
// Use the input from the ToF sensor to control a glowing ball
/***********************************************************************/
void ToFWipe(byte red, byte green, byte blue, int ballPosition) {
    uint32_t color = strip.Color(red, green, blue); 
    ballPosition = ballPosition % strip.numPixels();

    // Draw ending pads filling just the end 1/10th of the string
    strip.fill( color, 0, int(strip.numPixels()*.1) ); 
    strip.fill(color, (strip.numPixels() - (int(strip.numPixels()*.1))), strip.numPixels());

    // Draw the moving ball
    for (int j = 0; j < (strip.numPixels() *.1); j++ ) {  // Fill in about 2% of the string 
        strip.setPixelColor(ballPosition+j, Wheel((ballPosition) & 255));
    }

    strip.show();
    strip.clear();
}


/***********************************************************************/
// ToF Color
// Use the input from the ToF sensor to control a glowing ball
/***********************************************************************/
void ToFColor(int range) {
    
    for(int i=0; i < (strip.numPixels()); i++) {
        strip.setPixelColor(i, Wheel((range) & 255)); i=i+lessLight;
    } 

//    strip.fill(Wheel((range) & 255), 0, strip.numPixels()); 

    strip.show();
    strip.clear();
}

/***********************************************************************/
// Read the accelerometer to monitor for motion
/***********************************************************************/
int ReadAccel() {
    lis.read();      // get X Y and Z data at once
    sensors_event_t event;   /* Or....get a new sensor event, normalized */
    lis.getEvent(&event);
    // Then print out the raw data
    if (testMode >= 2) {
        Serial.print("Xr:  "); Serial.print(abs(lis.x));
        Serial.print("  \tYr:  "); Serial.print(abs(lis.y));
        Serial.print("  \tZr:  "); Serial.println(abs(lis.z));
    }

    if (testMode >= 1) {
        sensors_event_t event;
        lis.getEvent(&event);
        
        /* Display the results (acceleration is measured in m/s^2) */
        Serial.print("Xa: "); Serial.print(event.acceleration.x);
        Serial.print(" \tYa: "); Serial.print(event.acceleration.y);
        Serial.print(" \tZa: "); Serial.print(event.acceleration.z);
        Serial.println(" m/s^2 ");
    }
       
    int accelMagnitude = (abs(lis.x) + abs(lis.y) + abs(lis.z));  // normallize the movement such that sitting on a table is ~7600.
    int accelMagnitude2 = (abs(event.acceleration.x) + abs(event.acceleration.y) + abs(event.acceleration.z));  // get the cummaltive accelleration.

    if (testMode >= 1) {
      Serial.println(accelMagnitude);
      Serial.println(accelMagnitude2);
    }

    return accelMagnitude2;
    
}

/***********************************************************************/
// Read the Time of Flight distance to override program
/***********************************************************************/
int ReadToF() {
    uint8_t range = 0;  

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    // If the ToF sensor status returns an error, such as not ready, or over range, set the ToF program to 0    
    if (measure.RangeStatus == 4) {  // phase failures have incorrect data
        if (testMode >= 2) {Serial.println("ToF Out of Range");}
    } else if ( measure.RangeMilliMeter < 3000) {
        range = measure.RangeMilliMeter;
        
        if (testMode >= 2) {
            char buffer[255];
            sprintf(buffer, "Range: %d",
                    measure.RangeMilliMeter);
            Serial.println((char*)buffer);
        }
    }
	
    return range;
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

    // Logging control
    uint8_t logToSerial = 1;
    
    // timer statics for measuring time since last action
    static unsigned long previousLedUpdateMillis = 0;
    static unsigned long previousTransmitMillis = 0;
    static unsigned long previousPriorityUpdateMillis = 0;
    static unsigned long previousHeartBeatUpdateMillis = 0;

    // timer statics for checking Accel
    static unsigned long previousAccelCheckMillis = millis();
//    const unsigned long MovementThreshold = 13000; // Movement is normallized such that sitting on a table ~7600-12000
    const unsigned long MovementThreshold = 15; // Acceleration in m/s^2, so should be normallized such that sitting on a table ~9.8, int to 10, account for some error and set to 15
    const unsigned long AccelCheckPeriodMs = 50; // Update time between checking accel to see if we are moving
    const unsigned long notMovingTimeout = 5*60*1000; // how long to wait before giong to still program in ms
    static int notMovingTimer = 0; // timer for how many non-moving accelerometer measurements have been made
    const int StillProgram = 23; // pick a program to run when we are still

    // Time of Flight settings
    const int ToFProgram = 24; //min inclusive, max exclusive; // pick a program to run when we are overriding with ToF sensor
    static int range = 0; // Read the ToF sensor and use the range for changing the program
    static int localRange = 0; // Used for remote units to send their range
    static int requestedRemoteRange = 0; // Used for remote units to send their range

    // Broadcast timing
    const unsigned long transmitPeriodMs = 250; // how long to wait between broadcasts in ms
    const int minRssiThreshold = -99;  // recieve threshold
    const unsigned long overrideCoastMs = 5000;
    static int timeSinceGlobalOverride = 0;
    
    // led program controls  
    const unsigned long heartBeatLedPeriodMs = 100; // period for flashing the heartbeat LED
    const unsigned long priorityDecrementPeriodMs = 1;  // decrement the priority every X milliseconds
    const unsigned long minimumProgramTimeMs = 10000;  // How long to run a program after switching programs
    
    const uint8_t numLedPrograms = 22; // max case id, not count

    static uint8_t globalOverrideProgram = 0; // If something is overriding the program, like a sensor, this program will also be broadcast so other units sync to it
    static int8_t requestedRemoteGlobalOverride = 0; // If we recieve an override from a different unit
    static uint8_t localOverrideProgram = 0; // This local overriding the program, like when we aren't moving, will NOT be broadcast. Other units will not sync to it.
    
    const uint8_t defaultLedProgram = 0;
    static uint8_t overrideProgram = 0; // For testing specific paterns.
    
    static uint8_t ledProgram = defaultLedProgram;
    static uint8_t previuosLedProgram = defaultLedProgram;
    static uint8_t currentLedProgram = defaultLedProgram;
    static uint8_t previousLedProgram = defaultLedProgram;
    static uint8_t requestedLedProgram = defaultLedProgram;
    
    static int32_t currentProgramPrioity = minimumProgramTimeMs; // need to be allowed to go negative
    static int32_t requestedProgramPrioity = 0;
    
    static uint32_t ledUpdatePeriodMs = 10;  // this is delay waited before looping back through the LED case. A longer time here means the LEDs stay static with the current string display. This also blocks looking for recieved packets.
    
    /***********************************************************************/
    // Flash the heartbeat led to show the program is still running
    /***********************************************************************/
    if(millis() - previousHeartBeatUpdateMillis >= heartBeatLedPeriodMs){
        // Create a heartbeat LED flash to show we're updating the priority
        digitalWrite(LED_PIN, isLEDOn);
        isLEDOn = !isLEDOn;
        previousHeartBeatUpdateMillis = millis();
    }
    
    /***********************************************************************/
    // check Accel to see if we are moving
    /***********************************************************************/
    if (millis() - previousAccelCheckMillis >= AccelCheckPeriodMs) {
        
    		if (useAccel >= 1){
            
            // get the current accel data
            int accelMagnitude = ReadAccel();
            // Serial.print(" ReadAccel: "); Serial.print(accelMagnitude); Serial.println();
            // If the accel data is valid and we have significant movement, time how long we haven't been moving
            if ((accelMagnitude <= MovementThreshold) && (accelMagnitude > 0)) {
                notMovingTimer = (int)(notMovingTimer + (millis() - previousAccelCheckMillis));
            } else {
                notMovingTimer = 0;  // reset the counter since we are moving again
                localOverrideProgram = 0;
            }
            
            // If we haven't been moving for a long time, override the program
            if (notMovingTimer > notMovingTimeout) {
                if (ledProgram != localOverrideProgram) {  // log that we are not moving if this is the first time here
                    if(logToSerial == 1){
                        char buffer[255];
                                sprintf(buffer, "%ld %d %d %d %d: Motion timeout still for %d. LocalOverridePrg: %d",
                                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                                        notMovingTimer, localOverrideProgram);
                        Serial.println((char*)buffer);
                    }
                }
                localOverrideProgram = StillProgram; // go to a low power sparkly program
            }
    		}

        // update accel check timestamp
        previousAccelCheckMillis = millis();
		
 		    // While we are here... Check the Time of Flight sensor for possible overrides
    		if (useToF > 0) {
      			localRange = ReadToF();
        		
        		if (localRange > 0) {
                globalOverrideProgram = ToFProgram;
                range = localRange;
                if(logToSerial == 1){
		        char buffer[255];
          		        sprintf(buffer, "%ld %d %d %d %d: Time Of Flight Override Prg: %d localRange: %d",
                              millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                              globalOverrideProgram, localRange);
                      Serial.println((char*)buffer);
                }
            } else { 
              globalOverrideProgram = 0 ;
              range = 0 ;
            }
    		}
        
        // While we are here... Check for an analog input that may want to override
        if (useAnalog > 0) {            
            int analogInput1 = analogRead(ANALOG1);  // Gives a value from 0-1024
            
           if (analogInput1 < 100) { globalOverrideProgram = 0 ;}//do nothing... default to the synced program, or the override // 
           if (analogInput1 > 200) { globalOverrideProgram = 5; }//  rainbow 
           if (analogInput1 > 300) { globalOverrideProgram = 15; }// Meteor
           if (analogInput1 > 400) { globalOverrideProgram = 16; }// Strobe
           if (analogInput1 > 500) { globalOverrideProgram = 17; }// Fire
           if (analogInput1 > 600) { globalOverrideProgram = 19; }// Sparkle
           if (analogInput1 > 700) { globalOverrideProgram = 20; }// Sutro
           if (analogInput1 > 800) { globalOverrideProgram = 12; }// China Police mode
           if (analogInput1 > 900) { globalOverrideProgram = 21; }// Slow Sparkle
        }
        
        if ( requestedRemoteRange != 0) { range = requestedRemoteRange; }  // If we have a remote range, use it.        
    }
    
    /***********************************************************************/
    // Program priority update
    // priority is simply how long to play the program in ms
    /***********************************************************************/
    if(millis() - previousPriorityUpdateMillis >= priorityDecrementPeriodMs){
        // Update the current priority based on how long it's been runnging.
        currentProgramPrioity = (int32_t)((uint32_t)currentProgramPrioity - (millis() - previousPriorityUpdateMillis));
        
        // Update priority update timestamp
        previousPriorityUpdateMillis = millis();
        
        // if(logToSerial == 1){
        //        char buffer[255];
        //        uint16_t priorityUpdateLateMs = (uint16_t)((millis() - previousPriorityUpdateMillis) - priorityDecrementPeriodMs);
        //        sprintf(buffer, "%ld %d %d %d: ledDelay: %dms; priDecLate: %dms stillTime: %d",
        //                millis(), currentLedProgram, currentProgramPrioity, requestedProgramPrioity
        //                ledUpdatePeriodMs, priorityUpdateLateMs, notMovingTimer);
        //        Serial.println((char*)buffer);
        // }
        
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
    // if the program changes, we want to run each program for at least a min time,
    // so we set the new priority to the requested priorty plus the min time
    /***********************************************************************/
    // check to see if there is a new program requested
    // and that the request has a higher priority than our current priority
    if(requestedProgramPrioity > currentProgramPrioity ) {
        char buffer[255];
        
        // if the program is actually changing, reset the state trackers
        if (currentLedProgram != requestedLedProgram){
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: Changing program %d > %d",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        requestedProgramPrioity, currentProgramPrioity
                        );
                Serial.println((char*)buffer);
            }
            
            // give the new program a blank slate to play with, but only if we aren't using an override
            if (localOverrideProgram == 0 && requestedRemoteGlobalOverride == 0 && globalOverrideProgram == 0 && overrideProgram ==0) { 
                resetAllLedProgramStates();
            }
            
            // set the priority so it runs at least as long as our minimum
            currentProgramPrioity = requestedProgramPrioity + minimumProgramTimeMs;
            
            // change the led program
            currentLedProgram = requestedLedProgram;
            previousLedProgram = currentLedProgram;
            
        }
        else {
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: Syncing priority to %d from %d",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        requestedProgramPrioity, currentProgramPrioity
                        );
                Serial.println((char*)buffer);
            }
            
            // if we're not changing programs, just sync the priorities
            currentProgramPrioity = requestedProgramPrioity;
        }
        
        // reset the requested info
        requestedProgramPrioity = 0;
        requestedLedProgram = 0;
        
    }
    

    /***********************************************************************/
    // Led overrides
    // We can have a global override that we want to broadcast, or local override that only affects this unit, or a received global override
    // Order or priorty: 
    //    - globalOverrideProgram created on this unit
    //    - requestedRemoteGlobalOverride received from radio
    //    - localOverrideProgram (ie not-moving sparkle)
    //    - then lastly the random patterns synced between units
	  // We don't change the "currentLedProgram" nor "currentProgramPrioity" as we want to stay synced with other units. 
	  // We only chnage what we are displaying by changing "ledProgram".
    /***********************************************************************/

    previuosLedProgram = ledProgram;
  	ledProgram = currentLedProgram; // Set the program to play to the sync'ed current program
  	if ( localOverrideProgram !=0 ) { ledProgram = localOverrideProgram; }  // This override is generated locally, and only affects locally. Used for sleeping.
  	if ( requestedRemoteGlobalOverride !=0 ) {ledProgram = requestedRemoteGlobalOverride; }  // This override is received from other units. Used for other units with sensors that want to override everyone.
  	if ( globalOverrideProgram !=0 ) { 
  	    ledProgram = globalOverrideProgram;  // This override is generated locally, but we want to transmit to all other units.
    }  
    if ( overrideProgram != 0 ) { ledProgram = overrideProgram;}  // use the manual override if it exists for testing patterns
  		
    // Log if we have any newly added overrides 
    if ( ledProgram != previuosLedProgram ) { 
        // Looks like we are adding an override, or changing overrides. Clear previous program states.
        resetAllLedProgramStates();  
        
        // if this is the first time here, log the override program  
            if(logToSerial == 1){
                char buffer[255];
            sprintf(buffer,"%ld %d %d %d %d: Changing to %d",
                    millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                    ledProgram);
                Serial.println((char*)buffer);
            }
        }
        
    /***********************************************************************/
    // Led update
    // update the LEDs based on the current program
    /***********************************************************************/
    if (millis() - previousLedUpdateMillis >= ledUpdatePeriodMs){
        // update the previous time record
        previousLedUpdateMillis = millis();
        
        // play the current LED program
        switch(ledProgram){
            default:
                if(logToSerial == 1){
                    Serial.println("unknown LED program");
                }
                // fall through to use 0 as default
            case 0: // rainbow color wipe colorWipeRainbow ( colorSeed )
                ledUpdatePeriodMs = 1 * ledUpdateScaler;
                colorWipeRainbow (currentProgramPrioity);
                break;
            case 1: // random color wipe colorWipeRandom ( colorSeed )
                ledUpdatePeriodMs = 1 * ledUpdateScaler;
                colorWipeRandom (currentProgramPrioity);
                break;
            case 2: // rainbow
                ledUpdatePeriodMs = 1 * ledUpdateScaler;
                rainbow(); // rainbow
                break;
            case 3: // rainbowCycle
                ledUpdatePeriodMs = 10 ;
                rainbowCycle();
                break;
            case 4: // random color chase
                ledUpdatePeriodMs = 50 ;
                theaterChaseRandom(currentProgramPrioity);
                break;
            case 5: // MORE Sparkle Rainbow (byte red, byte green, byte blue, int sparksPerFlash, int sparkleDelay, int endPause) 
                // variable update rate based on state of the program
                ledUpdatePeriodMs = SparkleRainbow(currentProgramPrioity, 10, 30, 30);
                break;
            case 6: // color chase
                ledUpdatePeriodMs = 50 ;
                theaterChaseRainbow();
                break;
            case 7: // police mode
                // variable update rate based on state of the program
                ledUpdatePeriodMs = policeMode();
                break;
            case 8: // china poice mode
                // variable update rate based on state of the program
                ledUpdatePeriodMs = policeChinaMode2(15,25,300, false); // china Police Mode variable: wait between switching colors
                break;
            case 9: // color wipe whte and fade our red to reduce power
                ledUpdatePeriodMs = 1 * ledUpdateScaler;
                colorWipeAndFade (150, 120, 150);
                break;
            case 10:  // Strobe!
                // variable update rate based on state of the program
                // params: byte red, byte green, byte blue, int strobeCount, int flashDelay, int endPause
                ledUpdatePeriodMs = Strobe(150,100,200, 10, 25, 500);
                break;
                
            case 11: // Fire! variables: int Cooling, int Sparking, int SpeedDelay
                ledUpdatePeriodMs = 10 ;
                Fire(50,200);
                break;
               
            case 12:  // Running lights variables: byte red, byte green, byte blue, int WaveDelay
                ledUpdatePeriodMs = 1 * ledUpdateScaler;
                RunningLights(0,150,150);
                break;
            case 13:
                ledUpdatePeriodMs = StrobeRainbow(currentProgramPrioity, 10, 25, 500);
                break;

            case 14: // Sutro
                ledUpdatePeriodMs = 2 * ledUpdateScaler;
                Sutro();
                break;
                
            case 15: // purple color wipe
                ledUpdatePeriodMs = 1 * ledUpdateScaler;
                colorWipe (0, 75, 75);
                break;
 
            case 16: // MORE Sparkle (byte red, byte green, byte blue, int sparksPerFlash, int sparkleDelay, int endPause) 
                // variable update rate based on state of the program
                ledUpdatePeriodMs = Sparkle(255, 255, 255, 10, 30, 30);
                break;
            case 17: // china poice mode Half of the strip
                // int StrobeCount, int FlashDelay, int EndPause, bool halfString
                ledUpdatePeriodMs = policeChinaMode2 (15, 25, 300, true); // china Police Mode Half and Half variable:
                break;
            case 18:  // Running lights variables: byte red, byte green, byte blue, int WaveDelay
                ledUpdatePeriodMs = 10 + ledUpdateScaler;
                RunningLights(0,150,150);
                break;
            case 19:  // RainbowStick puts a rainbow on the stick and wiggles it variables: currentProgramPrioity used to get a sync rotating color
                ledUpdatePeriodMs = 10 ;
                RainbowStick(currentProgramPrioity);
                break;
            case 20:  // RainbowFill Fills the entire stick with one color, then changes the color variables: currentProgramPrioity used to get a sync rotating color
                ledUpdatePeriodMs = 10 ;
                RainbowFill(currentProgramPrioity);
                break;
            case 21: // meteorRainbow variables: colorSeed,  meteorSize,  meteorTrailDecay, boolean meteorRandomDecay
                ledUpdatePeriodMs = 1 * ledUpdateScaler ;
                meteorRainbow(currentProgramPrioity, 15, 70, true);
                break;
            case 22: // Ukraine
                ledUpdatePeriodMs = 2 * ledUpdateScaler;
                Ukraine();
                break;
                
// These programs are left out of the numLedPrograms so they are only used for overrides
// Sparkle slow is used for when there is no motion
            case 23: // Sparkle with Flag slow
                // uint32_t SparkleFlag(byte red, byte green, byte blue, int sparksPerFlash, int sparkleDelay, int endPause, flagColor) {
                // variable update rate based on state of the program
                ledUpdatePeriodMs = SparkleFlag(200, 225, 225, 2, 30, 250, currentProgramPrioity);
                break;
            case 24: // SparkleDecay
                // variable update rate based on state of the program (int red, int green, int blue, int fadeDelay, int endPause) {
                ledUpdatePeriodMs = SparkleDecay(100, 150, 150, 2, 0);
                break;




//            //
//            // Time Of Flight programs commented out for Bike Sabers where there isn't a ToF Sensor
//            //
//             case 25: // ToFWipe
//                // variable update rate based on state of the program
//                ledUpdatePeriodMs = 10;
//                ToFWipe(100,255,200, range);
//                break;
//            case 26: // ToFColor
//                // variable update rate based on state of the program
//                ledUpdatePeriodMs = 10;
//                ToFColor(range);
//                break;
//            case 27: // Fire! variables: int Cooling, int Sparking, int SpeedDelay
//                ledUpdatePeriodMs = 15;
//                Fire((range & 255),200);
//                break;

//            //
//            // These programs cause lockups. 
//            //
// 
//            case 10: // meteorRainbom variables: colorSeed,  meteorSize,  meteorTrailDecay, boolean meteorRandomDecay
//                ledUpdatePeriodMs = 1 * ledUpdateScaler;
//                meteorRainbow(currentProgramPrioity, 15, 70, true);
//                break;
//            case 16: // meteor variables: red,  green,  blue,  meteorSize,  meteorTrailDecay, boolean meteorRandomDecay
//                ledUpdatePeriodMs = 1 * ledUpdateScaler;
//                meteorRain(100,255,200, 10, 70, true);
//                break;

        }
    }
    
    /***********************************************************************/
    // Packet receive
    // Check if there is a new packet available, if there is, and the rssi
    // is above a threshold, decode it and
    // use it as the requested program and priority
    /***********************************************************************/
    if (testMode == 10) {  // etra logging to see how often we are checking for packets
        if(logToSerial == 1){
            char buffer[255];
            sprintf(buffer, "%ld %d %d %d: Looking for Received Packets",
                    millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity);
            Serial.println(buffer);
        }
    }

    if (rf69.available()){
        char buffer[255];
        uint8_t packet[RH_RF69_MAX_MESSAGE_LEN];
        int lastRssi = -100;
        uint8_t len=255;

        memset(packet, 0, RH_RF69_MAX_MESSAGE_LEN);
        memset(buffer, 0, 255);
        
        // read in the packet
        // note that the packet length does not appear to be valid in most cases
        bool validPacket = rf69.recv((uint8_t*)packet, &len);
        //null terminate just in case since we treat this like a char*
        packet[len] ='\0';
        
        if (validPacket && len > 3) {
            lastRssi = rf69.lastRssi();
            
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: Rxd. RSSI: %d len: %d \"%s\" ",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        lastRssi,
                        len, (char*)packet );
                Serial.println(buffer);
            }

            char *data = (char*)packet;
            int tempProgram = 0;
            int tempPriority = 0;
            int tempGlobalOverrideProgram = 0;
            int tempRange = 0;
            int numFound = 0;
            
            numFound = sscanf(data, "%d %d %d %d", &tempProgram, &tempPriority, &tempGlobalOverrideProgram, &tempRange);
            
            // if we got two items parsed out of the packet, use them for req
            if (numFound == 4){
                if(logToSerial == 1){
                    sprintf(buffer, "%ld %d %d %d %d: Decoded: %d %d %d %d",
                            millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                            tempProgram, tempPriority, tempGlobalOverrideProgram, tempRange);
                    Serial.println(buffer);
                }              
                
                // Populate the requestedRemoteGlobalOverride and requestedRemoteRange, which will also set it to 0 if the remote override has gone away.
                requestedRemoteGlobalOverride = (uint8_t) tempGlobalOverrideProgram;
                requestedRemoteRange = (uint8_t) tempRange;
                
                if ( requestedRemoteGlobalOverride != 0 ) { // if there's a global override, stop transmitting.
                    transmitMode = 0;

                    notMovingTimer = 0;  // reset the notMoving counter so we stay awake after a global override
                    localOverrideProgram = 0;
                    
                    timeSinceGlobalOverride = millis(); 
                    if(logToSerial == 1){
                        sprintf(buffer, "%ld %d %d %d %d: Global Override rx'd reqPri %d %d %d",
                                millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                                requestedLedProgram, requestedProgramPrioity, requestedRemoteGlobalOverride);
                        Serial.println(buffer); 
                    }
                } else {                     
                    if ( (millis() - timeSinceGlobalOverride > overrideCoastMs) && timeSinceGlobalOverride !=0 ) { 
                        transmitMode = 1;
                        timeSinceGlobalOverride = 0;
                        if(logToSerial == 1){
                            sprintf(buffer, "%ld %d %d %d %d: Global Override Coast Done. %ld Returning to regular program.",
                                    millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                                    timeSinceGlobalOverride);
                            Serial.println(buffer);
                        }
                    }
                } 

                // check the packet's rssi
                if(lastRssi > minRssiThreshold){
                    
                    // if the new packet has a higher priority than our curren req
                    // use it.
                    if(tempPriority > requestedProgramPrioity){
                        requestedProgramPrioity =  (int32_t) tempPriority;
                        requestedLedProgram =  (uint8_t) tempProgram;
                        if(logToSerial == 1){
                            sprintf(buffer, "%ld %d %d %d %d: Using rx'd reqPri %d %d %d",
                                    millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                                    requestedLedProgram, requestedProgramPrioity, requestedRemoteGlobalOverride);
                            Serial.println(buffer);
                        }
                    }
                    
                    // if the recieved packet has the same program, try to sync the priority
                    if(currentLedProgram == tempProgram){
                        currentProgramPrioity = tempPriority;
                    }

                }
                else {
                    if(logToSerial == 1){
                        sprintf(buffer, "%ld %d %d %d %d: Bad decode. Found %d items. \"%s\"",
                                millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                                numFound, data);
                        Serial.println(buffer);
                    }
                } // end if rssi threshold
            } // end if num found
        } // end if recv packet
        else {
            if(logToSerial == 1){
                char buffer[255];
                sprintf(buffer, "%ld %d %d %d %d: Bad rx. len: %d; \"%s\"", 
                millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        len, (char*)packet);
                Serial.println(buffer);
            }
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
    if (millis() - previousTransmitMillis >= transmitPeriodMs){
        char buffer[255];
        
        if (transmitMode == 1) {
            
            char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
            memset(radiopacket, 0, RH_RF69_MAX_MESSAGE_LEN);
            
            // generate a random new program with random priority
            int32_t tempPriority = (int32_t)(random(minimumProgramTimeMs, minimumProgramTimeMs*3/2));
            uint8_t tempProgram = (uint8_t)random(0, numLedPrograms + 1); //min inclusive, max exclusive
            
            // if the transmitted priority is higher, use it as the new requested
            if(tempPriority > requestedProgramPrioity){
                requestedProgramPrioity = tempPriority;
                requestedLedProgram = tempProgram;
                if(logToSerial == 1){
                    sprintf(buffer, "%ld %d %d %d %d: Using local reqPri %d %d",
                            millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                            requestedLedProgram, requestedProgramPrioity);
                    Serial.println(buffer);
                }
            }
            
            // if the current pri is higher than the temp pri (within some window)
            // send the current pri to try to get other to sync program _and currPri_ with us
            if( currentProgramPrioity > tempPriority){
                tempPriority = currentProgramPrioity;
                tempProgram = currentLedProgram;
            }
            
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: Sending %d %d %d %d; last tx: %dms (%d late) accel: %d",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        tempProgram, tempPriority, globalOverrideProgram, localRange,
                        (int)(millis() - previousTransmitMillis),
                        (int)((millis() - previousTransmitMillis) - transmitPeriodMs),
                        ReadAccel()
                        );
                Serial.println(buffer);
            }

            // try going into idle mode first to try not to hand the radio
            rf69.setModeIdle();
            
            sprintf(radiopacket, "%d %ld %d %d", tempProgram, tempPriority, globalOverrideProgram, localRange);
            // Send a message!
            rf69.send((uint8_t*)radiopacket, strlen(radiopacket));
            rf69.waitPacketSent(50);
            
            // put the radio back in rx mode
            // go through idle mode to try to clear any rx buffers
            rf69.setModeIdle();
            rf69.setModeRx();
            
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: Sent.",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity);
                Serial.println(buffer);
            }
            
        } else {
            // Log that we we aren't transmitting. Normally used when testing just one unit.
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: No tx transmitMode != 1, last tx: %dms (%d late)",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        (int)(millis() - previousTransmitMillis),
                        (int)((millis() - previousTransmitMillis) - transmitPeriodMs )
                        );
                Serial.println(buffer);
            }
        }
        
        if(testMode == 1 ) {
            requestedLedProgram = currentLedProgram+1;
            if (requestedLedProgram >= numLedPrograms) {requestedLedProgram = 0;}
            
            requestedProgramPrioity = minimumProgramTimeMs;
            
            if(logToSerial == 1){
                sprintf(buffer, "%ld %d %d %d %d: Test Mode req:%d %d",
                        millis(), currentLedProgram, ledProgram, currentProgramPrioity, requestedProgramPrioity,
                        requestedLedProgram, requestedProgramPrioity);
                Serial.println(buffer);
            }
        }
        
        previousTransmitMillis = millis();  // update time since last transmitted to now
    }

    
}
