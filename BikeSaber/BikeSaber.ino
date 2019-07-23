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

// Define structures and classes


// Define variables and constants
int lessLight = 0;  // use this for longer strings. It will disable every other LED on brighter programs to limit power.
int transmitMode = 0;  // use this for BikeSabers that we only want to recieve, but not vote.

int testMode = 1;     // If testing with just one BikeSaber, use this mode which: moves to the next program sequentially


// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions

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
#define NUMPIXELS 155
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
    
    // Setup the NeoPixel string
    strip.begin(); // This initializes the NeoPixel library.
    strip.show(); // start with everything off
    
}



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


// Fill the dots one after the other with a color
uint16_t colorWipecurrentPixel = 0;
void colorWipe(uint32_t c) {
    strip.setPixelColor(colorWipecurrentPixel, c);
    strip.show();
    colorWipecurrentPixel++;
    if(colorWipecurrentPixel >= strip.numPixels()+50){
        // we've filled the strip with c, now turn it all off and start back at pixel 0
        for(int i=0; i < strip.numPixels(); i++){
          strip.setPixelColor(i, strip.Color(0, 0, 0));
          strip.show();
        }
        strip.show();
        colorWipecurrentPixel = 0;
    }
}

// Flash Red-Blue on the full string
int policePreviousColor = 0;
void policeMode(uint8_t wait) {
     switch(policePreviousColor){

        case 0: // red color wipe
            policePreviousColor++;
            for(int i=0; i < (strip.numPixels()); i++){
                 strip.setPixelColor(i, strip.Color(0, 150, 0));
            }
            strip.show();
            delay(wait);
            break;

        case 1: // green color wipe
            policePreviousColor=0;
            for(int i=0; i < (strip.numPixels()); i++){
                 strip.setPixelColor(i, strip.Color(0, 0, 150));
            }
            strip.show();
            delay(wait);
            break;
     }
}


// China flashing Red-Blue on the full string
int policeChinaPreviousColor = 0;
void policeChinaMode(uint8_t wait) {

        // red color wipe
        if(policeChinaPreviousColor == 0 || policeChinaPreviousColor == 2){
            policeChinaPreviousColor++;
            for(int i=0; i < (strip.numPixels()); i++){
                 strip.setPixelColor(i, strip.Color(0, 200, 0));
                 }
            strip.show();
            delay(wait);
        }
        
        // off wipe
        if(policeChinaPreviousColor == 1 || 
        policeChinaPreviousColor == 3 || policeChinaPreviousColor == 4 || policeChinaPreviousColor == 5 || 
        policeChinaPreviousColor == 7 || 
        policeChinaPreviousColor == 9 || policeChinaPreviousColor == 10 || policeChinaPreviousColor == 11){
        policeChinaPreviousColor++;
            for(int i=0; i < (strip.numPixels()); i++){
                 strip.setPixelColor(i, strip.Color(0, 0, 0));
                }
            strip.show();
            delay(wait*4);
        }

        // blue color wipe
        if(policeChinaPreviousColor == 6 || policeChinaPreviousColor == 8){
            policeChinaPreviousColor++;
            for(int i=0; i < (strip.numPixels()); i++){
                 strip.setPixelColor(i, strip.Color(0, 0, 200));
                }
            strip.show();
            delay(wait);
        }
        
        // reset 
        if( policeChinaPreviousColor == 12) {policeChinaPreviousColor=0;}
       
}

// China flashing Red-Blue on the half string
int policeChinaHalfPreviousColor = 0;
void policeChinaModeHalf(uint8_t wait) {
        
        int NUMPIXELS_HALF = (strip.numPixels() / 2);
        
        // red color wipe
        if(policeChinaHalfPreviousColor == 0 || policeChinaHalfPreviousColor == 2){
            policeChinaHalfPreviousColor++;
            for(int i=0; i < NUMPIXELS_HALF; i++){  
                 strip.setPixelColor(i, strip.Color(0, 255, 0));
            }
            strip.show();
            delay(wait);
        }
        
        // off wipe
        if(policeChinaHalfPreviousColor == 1 || 
        policeChinaHalfPreviousColor == 3 || policeChinaHalfPreviousColor == 4 || policeChinaHalfPreviousColor == 5 || 
        policeChinaHalfPreviousColor == 7 || 
        policeChinaHalfPreviousColor == 9 || policeChinaHalfPreviousColor == 10 || policeChinaHalfPreviousColor == 11){
        policeChinaHalfPreviousColor++;
            for(int i=0; i < strip.numPixels(); i++){
                 strip.setPixelColor(i, strip.Color(0, 0, 0));
            }
            strip.show();
            delay(wait*4);
        }

        // blue color wipe
        if(policeChinaHalfPreviousColor == 6 || policeChinaHalfPreviousColor == 8){
            policeChinaHalfPreviousColor++;
            for(int i=NUMPIXELS_HALF; i < strip.numPixels(); i++){
                 strip.setPixelColor(i, strip.Color(0, 0, 255));
            }
            strip.show();
            delay(wait);
        }
        
        // reset 
        if( policeChinaHalfPreviousColor == 12) {policeChinaHalfPreviousColor=0;}
       
}

int policeChina2PreviousColor = 0;
void policeChinaMode2(int StrobeCount, int FlashDelay, int EndPause) { // int StrobeCount, int FlashDelay, int EndPause)

    if(policeChina2PreviousColor == 0) { 
          uint32_t strobeColor = strip.Color(0, 255, 0);  // flash red
          for(int j = 0; j < StrobeCount; j++) {
              strip.fill(strobeColor,0, NUMPIXELS);
              strip.show();
              delay(FlashDelay);
              strip.clear();
              strip.show();
              delay(FlashDelay);
          }        
       delay(EndPause);
       policeChina2PreviousColor++;
      }
 
    if(policeChina2PreviousColor == 1) { 
          uint32_t strobeColor = strip.Color(0, 0, 255);  // flash blue
          for(int j = 0; j < StrobeCount; j++) {
              strip.fill(strobeColor, 0, NUMPIXELS);
              strip.show();
              delay(FlashDelay);
              strip.clear();
              strip.show();
              delay(FlashDelay);
          }        
       delay(EndPause);
       policeChina2PreviousColor = 0;
      }
}

int policeChinaHalf2PreviousColor = 0;
void policeChinaModeHalf2(int StrobeCount, int FlashDelay, int EndPause) { // int StrobeCount, int FlashDelay, int EndPause)

    if(policeChinaHalf2PreviousColor == 0) { 
          uint32_t strobeColor = strip.Color(0, 255, 0);  // flash red
          for(int j = 0; j < StrobeCount; j++) {
              strip.fill(strobeColor,0, (NUMPIXELS/2));
              strip.show();
              delay(FlashDelay);
              strip.clear();
              strip.show();
              delay(FlashDelay);
          }        
       delay(EndPause);
       policeChinaHalf2PreviousColor++;
      }
 
    if(policeChinaHalf2PreviousColor == 1) { 
          uint32_t strobeColor = strip.Color(0, 0, 255);  // flash blue
          for(int j = 0; j < StrobeCount; j++) {
              strip.fill(strobeColor, (NUMPIXELS/2), NUMPIXELS);
              strip.show();
              delay(FlashDelay);
              strip.clear();
              strip.show();
              delay(FlashDelay);
          }        
       delay(EndPause);
       policeChinaHalf2PreviousColor = 0;
      }
}

uint16_t rainbowColorMotion = 0;
void rainbow( int wait ) {
    
//    for(j=0; j<256; j++) {
//        for(i=0; i<strip.numPixels(); i++) {
//            strip.setPixelColor(i, Wheel((i+j) & 255));
//        }
//        strip.show();
//    }

    for(uint16_t i=0; i<strip.numPixels(); i=i+1) {
        if ( lessLight ) { strip.setPixelColor(i, 0); i=i+1;}   //turn off every other pixel
        strip.setPixelColor(i, Wheel((i+rainbowColorMotion) & 255));

    }
    strip.show();
    delay (wait);
    rainbowColorMotion++;
    if(rainbowColorMotion > 255) rainbowColorMotion = 0;
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(int wait) {
//    for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
//        for(uint16_t i=0; i< strip.numPixels(); i++) {
//            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
//        }
//        strip.show();
//    }

    for(uint16_t i=0; i< strip.numPixels(); i=i+1) {
        if ( lessLight ) { strip.setPixelColor(i, 0); i=i+1;}   //turn off every other pixel
        strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + rainbowColorMotion) & 255));
    }

    strip.show();
    delay (wait);

    rainbowColorMotion++;
    if(rainbowColorMotion > 256*5) rainbowColorMotion = 0;  
}


//Theatre-style crawling lights.
void theaterChase(uint32_t c, int wait) {
//    for (int j=0; j<1; j++) {  //do 1 cycles of chasing
//        for (int q=0; q < 3; q++) {
//            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
//                strip.setPixelColor(i+q, c);    //turn every third pixel on
//            }
//            strip.show();
//
//            delay(20);
//
//            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
//                strip.setPixelColor(i+q, 0);        //turn every third pixel off
//            }
//        }
//    }
    static uint8_t q;

    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q-1, 0);    //turn off every third pixel from the previous run
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //set every third pixel
    }
    strip.show();
    delay (wait);

    // every other step, turn the leds off (alternating with the color c)
    q++;
    if(q >= 3) q = 0;

}


//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
//    for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
//        for (int q=0; q < 3; q++) {
//            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
//                strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
//            }
//            strip.show();
//
//            delay(wait);
//
//            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
//                strip.setPixelColor(i+q, 0);        //turn every third pixel off
//            }
//        }
//    }
    static uint8_t q;
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q-1, 0);        //turn every third pixel off
    }
    
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+rainbowColorMotion) % 255));    //turn every third pixel on
    }
    strip.show();
    delay (wait);

    // every other step, turn the leds off (alternating with the color c)
    q++;
    if(q >= 3) q = 0;
    
    rainbowColorMotion++;
    if(rainbowColorMotion > 255) rainbowColorMotion = 0;
}


//#####################################################
// From https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/
//#####################################################

// colorWipeAndBlack (0x00,255,0x00, 50);
// colorWipeAndBlack (0,0,0, 50); // wipe to black
// colorWipeAndBlack ((random(50,200)),(random(50,200)),(random(50,200)), 50);  // random color 

void colorWipeAndBlack(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      setPixel(i, red, green, blue);
      showStrip();
      delay(SpeedDelay);
  }
}

//############
//meteorRain(255,255,255, 10, 64, true, 30);

void meteorRain(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay) {  
	// Start with all off and start back at pixel 0
	strip.clear();
	strip.show();
  
  
   for(int i = strip.numPixels() + strip.numPixels(); i > 0; --i) {
    
    
    // fade brightness all LEDs one step
    for(int j=0; j< strip.numPixels(); j++) {
      if( (!meteorRandomDecay) || (random(10)>5) ) {
        fadeToBlack(j, meteorTrailDecay );        
      }
    }
    
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j < strip.numPixels() ) && ( i - j >=0 ) ) {
        strip.setPixelColor(i-j, red, green, blue);
      } 
    }
   
    strip.show();
    delay(SpeedDelay);
  }
}

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

//############
//  Strobe(255, 255, 255, 10, 50, 1000);

void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause){

    uint32_t strobeColor = strip.Color(red, green, blue);
    for(int j = 0; j < StrobeCount; j++) {
        strip.fill(strobeColor,0, strip.numPixels());
        strip.show();
        delay(FlashDelay);
        strip.clear();
	      strip.show();
        delay(FlashDelay);
    }
 
 delay(EndPause);
}

//#############
//  Fire(55,120,15);

void Fire(int Cooling, int Sparking, int SpeedDelay) {
  	strip.clear();
    strip.show();

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
  delay(SpeedDelay);
}

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

//#############

//  RunningLights(255,255,0, 50);

void RunningLights(byte red, byte green, byte blue, int WaveDelay) {
  int Position=0;
  
  for(int j = 0 ; j < strip.numPixels(); j++)
  {
      Position++; // = 0; //Position + Rate;
      for(int i=0; i < strip.numPixels(); i++) {
        // sine wave, 3 offset waves make a rainbow!
        //float level = sin(i+Position) * 127 + 128;
        //setPixel(i,level,0,0);
        //float level = sin(i+Position) * 127 + 128;
        setPixel(i,((sin(i+Position) * 127 + 128)/255)*red,
                   ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
      }
      
      strip.show();
      delay(WaveDelay);
  }
}

//################
//  Sparkle(255, 255, 255, 0, 10);

void Sparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay) {
  int Pixel = random(strip.numPixels());
  setPixel(Pixel,red,green,blue);
  setPixel((Pixel+1),red,green,blue);
  strip.show();
  delay(SparkleDelay);

  setPixel(Pixel,0,0,0);
  setPixel((Pixel+1),0,0,0);
  strip.show();
  delay(SpeedDelay);

}

//#################
void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < strip.numPixels(); i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}


void showStrip() {
   strip.show();
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

//#####################################################


void loop() {
    // Some example procedures showing how to display to the pixels:

//    colorWipe(strip.Color(0, 255, 0), 50); // Green
//    colorWipe(strip.Color(0, 0, 255), 50); // Blue
//
//    // Send a theater pixel chase in...
//    theaterChase(strip.Color(127, 127, 127), 50); // White
//    theaterChase(strip.Color(127, 0, 0), 50); // Red
//    theaterChase(strip.Color(0, 0, 127), 50); // Blue
//
//    rainbow(20);
//    rainbowCycle(20);
//    theaterChaseRainbow(50);
    
    static bool isLEDOn = false;
    
    // tiemr statics for measuring time since last action
    static unsigned long previousLedUpdateMillis = 0;
    static unsigned long previousTransmitMillis = 0;
    static unsigned long previousPriorityUpdateMillis = 0;
    
    // current time info
    unsigned long currentMillis = millis();
    
    // led program controls
    const uint8_t numLedPrograms = 20; // max case id, not count
    const uint8_t defaultLedProgram = 4;
    const uint8_t overrideProgram = 0; // for testing, we want a static program
    static uint8_t currentLedProgram = defaultLedProgram;
    static uint8_t previousLedProgram = defaultLedProgram;
    static uint8_t requestedLedProgram = defaultLedProgram;
    char comment[100] = "";
    static int16_t currentProgramPrioity = 50+50;
    static int16_t previousProgramPrioity = 0;
    static int8_t requestedProgramPrioity = 0;
    static unsigned long ledUpdatePeriodMs = 10;  // this is delay waited before looping back through the LED case. A longer time here means the LEDs stay static with the current string display. This also blocks looking for recieved packets.
    
    
    /***********************************************************************/
    // Program priority update
    /***********************************************************************/
    const unsigned long priorityDecrementPeriodMs = 100;  // decrement the priority every X milliseconds. 100 means decrement the priority every 10ms.
    const unsigned long minimumProgramTimeMs = 5000;  // Run the current program for atleast 2secs before looking for new program
    if(currentMillis - previousPriorityUpdateMillis >= priorityDecrementPeriodMs){
//        currentProgramPrioity = currentProgramPrioity == 0 ? 0 : --currentProgramPrioity;
          previousProgramPrioity = currentProgramPrioity;
          currentProgramPrioity = currentProgramPrioity == 0 ? 0 : (currentProgramPrioity - ((currentMillis - previousPriorityUpdateMillis)/priorityDecrementPeriodMs) );   
          previousPriorityUpdateMillis = currentMillis;

          digitalWrite(LED_PIN, isLEDOn);
          isLEDOn = !isLEDOn;
          
          char buffer[255];
          sprintf(buffer, "%ld: prg: %d pri: %d updateWait: %d ms", currentMillis, currentLedProgram, currentProgramPrioity, ledUpdatePeriodMs);
          Serial.println((char*)buffer);
          
    }
    
    
    /***********************************************************************/
    // Led update
    // update the LEDs based on the current program
    /***********************************************************************/
    if (currentMillis - previousLedUpdateMillis >= ledUpdatePeriodMs){
        // update the previous time record
        char buffer[255];

        
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
            // set the priority so it runs at least 5s
            currentProgramPrioity = requestedProgramPrioity + (minimumProgramTimeMs / priorityDecrementPeriodMs);
            
            // if the new program is the same as it was last time, increment so we get more changes
            if(currentLedProgram == previousLedProgram) currentLedProgram++;

            if (overrideProgram != 0) {   // if there is an override program number from a knob input, or accelerometer, use that program.
                currentLedProgram = overrideProgram;
            }
            
            // reset the requested info
            requestedProgramPrioity = 0;
            requestedLedProgram = 0;
            
            sprintf(buffer, "%ld: Changing to prg: %d pri: %d", currentMillis, currentLedProgram, currentProgramPrioity);
            Serial.println((char*)buffer);

            setAll (0,0,0); // clear all LEDs by setting them to off
//            rainbowColorMotion = 0;  // This should be removed?
        }
        else {
            // if there isn't a higher priority, run the previous program
            currentLedProgram = previousLedProgram;
        }
        
        switch(currentLedProgram){
            default:
                Serial.println("unknown LED program");
                // fall through to use 0 as default
            case 0: // red color wipe  variables: byte red, byte green, byte blue, wait before adding each LED
//                comment = "Wipe Red";
                ledUpdatePeriodMs = 0;
                colorWipeAndBlack (0, 150, 0, 2); // Red
                colorWipeAndBlack (0,0,0, 0); // wipe to black

                break;
            case 1: // green color wipe
 //               comment = "Wipe Green";
                ledUpdatePeriodMs = 0;
                colorWipeAndBlack (150, 0, 0, 2); // Grean
                colorWipeAndBlack (0,0,0, 0); // wipe to black
                break;
            case 2: // blue color wipe
 //               comment = "Wipe Blue";
                ledUpdatePeriodMs = 0;
                colorWipeAndBlack (0, 0, 150, 2); // Blue
                colorWipeAndBlack (0,0,0, 0); // wipe to black
                break;
            case 3: // purple color wipe
 //               comment = "Wipe Puruple";
                ledUpdatePeriodMs = 0;
                colorWipeAndBlack (0, 75, 75, 2); // Purple
                colorWipeAndBlack (0,0,0, 0); // wipe to black
                break;
            case 4: // rainbow
 //               comment = "Rainbow";
                ledUpdatePeriodMs = 10;
                rainbow(10); // rainbow
                break;
            case 5: // rainbowCycle
//                comment = "Rainbox Cycle";
                ledUpdatePeriodMs = 10;
                rainbowCycle(10); // rainbowCyle: delay
                break;
            case 6: // blue color chase
//                comment = "Chase Blue";
                ledUpdatePeriodMs = 10;
                theaterChase(strip.Color(0, 0, 255), 50); // Chase Blue
                break;
            case 7: // red color chase
//                comment = "Chase Red";
                ledUpdatePeriodMs = 10;
                theaterChase(strip.Color(0, 255, 0), 50); // Chase red
                break;
            case 8: // green color chase
//                comment = "Chase Green";
                ledUpdatePeriodMs = 10;
                theaterChase(strip.Color(255, 0, 0), 50); // Chase green
                break;
            case 9: // purple color chase
//                comment = "Chase Purple";
                ledUpdatePeriodMs = 10;
                theaterChase(strip.Color(0, 75, 75), 50); // Chase purple
                break;
            case 10: // color chase
//                comment = "Chase Rainbow";
                ledUpdatePeriodMs = 10;
                theaterChaseRainbow(50); // Chase rainbow
                break;
            case 11: // poice mode
//                comment = "Police Mode";
                ledUpdatePeriodMs = 1;
                policeMode(100); // Police Mode variable: wait between switching colors
                break;
            case 12: // china poice mode
//                comment = "Police Mode China";
                ledUpdatePeriodMs = 1;
                policeChinaMode2(10,20,300); // china Police Mode variable: wait between switching colors
                break;
           
            case 13: // china poice mode Half of the strip
//                comment = "Police Mode China Half";
                ledUpdatePeriodMs = 1;
                policeChinaModeHalf2 (10, 20, 300); // china Police Mode Half and Half variable: // int StrobeCount, int FlashDelay, int EndPause)
                break;
           
            case 14: // color wipe random color and back to black
//                comment = "Wipe Random Color";
                ledUpdatePeriodMs = 0;
                colorWipeAndBlack ((random(0,200)),(random(0,200)),(random(0,200)), 2);  // random color 
                colorWipeAndBlack (0,0,0, 1); // wipe to black
                break;
                
            case 15: // meteor variables: red,  green,  blue,  meteorSize,  meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay
//                comment = "Meteor";
                ledUpdatePeriodMs = 1;
                meteorRain(255,255,255, 10, 64, true, 5);
                break;
                
            case 16:  // Strobe! variables: byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause
//                comment = "Strobe";
                ledUpdatePeriodMs = 1;
                Strobe(0,100,255, 10, 25, 250);
                break;
                
            case 17: // Fire! variables: int Cooling, int Sparking, int SpeedDelay
//                comment = "Fire";
                ledUpdatePeriodMs = 0;
                Fire(55,120,15);
                break;
                
            case 18:  // Running lights variables: byte red, byte green, byte blue, int WaveDelay
//                comment = "Running Lights";
                ledUpdatePeriodMs = 10;
                RunningLights(0,150,150, 10);
                break;
                
            case 19: // Sparkle variables: byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay
//                comment = "Sparkle";
                ledUpdatePeriodMs = 0;
                Sparkle(255, 255, 255, 10, 5);
                break;
 
            case 20: // Sparkle variables: byte red, byte green, byte blue, int SpeedDelay
//                comment = "Sparkle Slow";
                ledUpdatePeriodMs = 0;
                Sparkle(255, 255, 255, 50, 500);
                break;
        }
        
        previousLedProgram = currentLedProgram;
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

    const int minRssiThreshold = -80;

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
            
            sprintf(buffer, "%ld: Packet len: %d \"%s\" Rssi: %d", currentMillis, len, (char*)packet, lastRssi);
            Serial.println(buffer);
            
            // check the packet's rssi
            if(lastRssi > minRssiThreshold){
                int tempProgram = 0, tempPriority = 0;
                int numFound = 0;
                
                numFound =  sscanf((char*)packet, "%d %d", &tempProgram, &tempPriority);
                
                // if we got two items parsed out of the packet, use them for req
                if (numFound == 2){
                    requestedProgramPrioity = (uint8_t) tempPriority;
                    requestedLedProgram = (uint8_t) tempProgram;
                    sprintf(buffer, "%ld: Got req: %d %d Rssi: %d", currentMillis, requestedLedProgram, requestedProgramPrioity, lastRssi);
                }
                else {
                    Serial.println("Bad packet");
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
    const unsigned long transmitPeriodMs = 1*1000; // 1s
      if (currentMillis - previousTransmitMillis >= transmitPeriodMs){

            if (transmitMode == 1) {

                char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
                char buffer[255];
                
                // generate a random new program with random priority
                requestedProgramPrioity = (int16_t)random(1, minimumProgramTimeMs / priorityDecrementPeriodMs);
                requestedLedProgram = (uint8_t)random(0, numLedPrograms
                                                      +1); //min inclusive, max exclusive
    
                                                      
                sprintf(radiopacket, "%d %d", requestedLedProgram, requestedProgramPrioity);
                
                sprintf(buffer, "%ld: Sending prg:%d pri:%d pack:\"%s\" len: %d Previous Transmit: %d ms", currentMillis, requestedLedProgram, requestedProgramPrioity, radiopacket, strlen(radiopacket), (currentMillis - previousTransmitMillis));
                Serial.println(buffer);
        
                // Send a message!
                rf69.send((uint8_t*)radiopacket, strlen(radiopacket));
                rf69.waitPacketSent();
            } else {
                  char buffer[255];

                  // If testing with just one unit, move to the next program so we can see more.
                  sprintf(buffer, "%ld: Not transmitting because transmitMode != 1, Previous Transmit: %d ms", currentMillis, (currentMillis - previousTransmitMillis));
                  Serial.println(buffer);


                  if(testMode == 1 ) { 
                        requestedLedProgram = currentLedProgram++; 
                        if (requestedLedProgram >= numLedPrograms+1) {requestedLedProgram = 0;}
                        
                        requestedProgramPrioity =  (minimumProgramTimeMs / priorityDecrementPeriodMs);
                        sprintf(buffer, "%ld: Test Mode prg:%d pri:%d ", currentMillis, requestedLedProgram, requestedProgramPrioity);
                  }
            }
            previousTransmitMillis = currentMillis;

    }

    
}



