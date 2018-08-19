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


// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions

// ***************************************************************************
// Stuff for RFM69
// ***************************************************************************
// Add setup code
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
#define RFM69_INT     3  //
#define RFM69_CS      4  //
#define RFM69_RST     2  // "A"
#define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
#define RFM69_CS      2    // "E"
#define RFM69_IRQ     15   // "B"
#define RFM69_RST     16   // "D"
#define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13
#endif

/* Teensy 3.x w/wing
 #define RFM69_RST     9   // "A"
 #define RFM69_CS      10   // "B"
 #define RFM69_IRQ     4    // "C"
 #define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
 */

/* WICED Feather w/wing
 #define RFM69_RST     PA4     // "A"
 #define RFM69_CS      PB4     // "B"
 #define RFM69_IRQ     PA15    // "C"
 #define RFM69_IRQN    RFM69_IRQ
 */

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// ***************************************************************************
// Stuff for LED string test
// ***************************************************************************
#define NUMPIXELS 302
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
    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    rf69.setEncryptionKey(key);
    
    pinMode(LED, OUTPUT);
    
    Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
    
    // Setup the NeoPixel string
    strip.begin(); // This initializes the NeoPixel library.
    strip.show(); // start with everything off
    
}

//void loop() {
//    unsigned long timeSent = 0;
//    delay(100);
//
//    char radiopacket[20] = "Hello World #";
//    itoa(packetnum++, radiopacket+13, 10);
//    Serial.print("Sending "); Serial.println(radiopacket);
//
//    // Send a message!
//    rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
//    rf69.waitPacketSent();
//    timeSent = micros();
//
//    // Now wait for a reply
//    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
//    uint8_t len = sizeof(buf);
//    int8_t lastRssi = -100;
//
//
//    //if (rf69.waitAvailableTimeout(500))  {
//    if (rf69.available()){
//        newPacketFlag = false;
//        unsigned long timeOfResponse = micros();
//        sprintf((char*)buf, "Got a reply after %d: ", timeOfResponse - timeSent);
//        Serial.print((char*)buf);
//        // Should be a reply message for us now
//        if (rf69.recv(buf, &len)) {
//            Serial.print((char*)buf);
//            Serial.print(" ");
//            lastRssi = rf69.lastRssi();
//            Serial.println(lastRssi, DEC);
//            Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
//        } else {
//            Serial.println("Receive failed");
//            lastRssi = -100;
//        }
//    } else {
//        Serial.println("No reply, is another RFM69 listening?");
//        lastRssi = -100;
//    }
//
//    static uint8_t frameNumber = 0;
//    frameNumber++;
//    frameNumber = frameNumber % 4;
//    for(int i=0;i<NUMPIXELS;i++){
//        int colors[2][4] = {{0x960000, 0x009600, 0x000096, 0x000000}, // R, G, B, 0
//                            {0x000015, 0x000050, 0x000096, 0x000050}}; // blue breath
//        int color = 0;
//        uint8_t currentColorPattern = 0;
//        static uint8_t previousColorPattern = 0;
//
//        // set the color pattern based on the last RSSI
//        if(lastRssi > -30){
//            currentColorPattern = 0;
//        }
//        else {
//            currentColorPattern = 1;
//        }
//
//        // if the color pattern changed, re-start the frame counter
//        // this will have the color patterns always start at the beginning.
//        if (previousColorPattern != currentColorPattern){
//            frameNumber = 0;
//        }
//        previousColorPattern = currentColorPattern;
//
//        color = colors[currentColorPattern][frameNumber];
//
//        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//        //pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright green color.
//        pixels.setPixelColor(i, pixels.Color((uint8_t)(color>>16),
//                                             (uint8_t)(color>>8),
//                                             (uint8_t)(color))); // Moderately bright green color.
//
//    }
//    pixels.show(); // This sends the updated pixel color to the hardware.
//}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}



// Fill the dots one after the other with a color
uint16_t colorWipecurrentPixel = 0;
void colorWipe(uint32_t c) {
    strip.setPixelColor(colorWipecurrentPixel, c);
    strip.show();
    colorWipecurrentPixel++;
    if(colorWipecurrentPixel > strip.numPixels()){
        // we've filled the strip with c, no turn it all off and start back at pixel 0
        for(int i=0; i < strip.numPixels(); i++){
          strip.setPixelColor(i, strip.Color(0, 0, 0));
        }
        strip.show();
        colorWipecurrentPixel = 0;
    }
}

uint16_t rainbowColorMotion;
void rainbow() {
    
//    for(j=0; j<256; j++) {
//        for(i=0; i<strip.numPixels(); i++) {
//            strip.setPixelColor(i, Wheel((i+j) & 255));
//        }
//        strip.show();
//    }

    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, Wheel((i+rainbowColorMotion) & 255));
    }
    strip.show();
    rainbowColorMotion = rainbowColorMotion > 255 ? 0 : rainbowColorMotion++;
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle() {
    uint16_t i, j;
    
    for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
        for(i=0; i< strip.numPixels(); i++) {
            strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        strip.show();
    }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
    for (int j=0; j<1; j++) {  //do 1 cycles of chasing
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, c);    //turn every third pixel on
            }
            strip.show();

            delay(wait);
            
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
    for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
        for (int q=0; q < 3; q++) {
            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
            }
            strip.show();
            
            delay(wait);          

            for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
                strip.setPixelColor(i+q, 0);        //turn every third pixel off
            }
        }
    }
}




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
    static uint8_t currentLedProgram = 0;
    static uint8_t previousLedProgram = 0;
    static uint8_t requestedLedProgram = 0;
    static int16_t currentProgramPrioity = 50+50;
    static int8_t requestedProgramPrioity = 0;
    
    
    /***********************************************************************/
    // Program priority update
    /***********************************************************************/
    const unsigned long priorityDecrementPeriodMs = 100;
    const unsigned long minimumProgramTimeMs = 5000;
    if(currentMillis - previousPriorityUpdateMillis >= priorityDecrementPeriodMs){
        previousPriorityUpdateMillis = currentMillis;
        currentProgramPrioity = currentProgramPrioity == 0 ? 0 : --currentProgramPrioity;
    }
    
    
    /***********************************************************************/
    // Led update
    // update the LEDs based on the current program
    /***********************************************************************/
    const unsigned long ledUpdatePeriodMs = 25;
    if (currentMillis - previousLedUpdateMillis >= ledUpdatePeriodMs){
        // update the previous time record
        previousLedUpdateMillis = currentMillis;
        
        char buffer[255];
        
        /***********************************************************************/
        // Program change
        // if there is a new program request, change to it if it has a higher
        // priority than our current priority
        // if the program changes, we want to run each program for at least 5s,
        /// so we set the new priority to the requested priorty plus 5 *priorityDecrementPeriodMs
        /***********************************************************************/
        // check to see if there is a new program requested
        // and that the request has a higher priority than our current priority
        if(requestedProgramPrioity > currentProgramPrioity){
            // change the led program
            currentLedProgram = requestedLedProgram;
            // set the priority so it runs at least 5s
            currentProgramPrioity = requestedProgramPrioity + (minimumProgramTimeMs / priorityDecrementPeriodMs);
            
            // reset the requested info
            requestedProgramPrioity = 0;
            requestedLedProgram = 0;
            
            sprintf(buffer, "Changing to prg: %d pri: %d", currentLedProgram, currentProgramPrioity);
            Serial.println((char*)buffer);
            // TODO: reset all the LED programs?
            colorWipecurrentPixel = 0;
            rainbowColorMotion = 0;
        }
        else {
            // if there isn't a higher priority, run the previous program
            currentLedProgram = previousLedProgram;
        }
        
        // if the program changed, reset the pixel programs
        if(currentLedProgram != previousLedProgram){
            colorWipecurrentPixel = 0;
        }
        
        digitalWrite(LED_PIN, isLEDOn);
        isLEDOn = !isLEDOn;
        //Serial.println(++i);
        //Serial.println(currentMillis);
        sprintf(buffer, "%ld: prg: %d pri: %d", currentMillis, currentLedProgram, currentProgramPrioity);
        Serial.println((char*)buffer);
        
        switch(currentLedProgram){
            default:
                Serial.println("unknown LED program");
                // fall through to use 0 as default
            case 0: // red color wipe
                colorWipe(strip.Color(255, 0, 0)); // Red
                break;
            case 1: // green color wipe
                colorWipe(strip.Color(0, 255, 0)); // Grean
                break;
            case 2: // blue color wipe
                colorWipe(strip.Color(0, 0, 255)); // Blue
                break;
            case 3: // yellow color wipe
                colorWipe(strip.Color(255, 205, 0)); // Yellow
                break;

            case 4: // rainbow
                rainbow(); // rainbow
                break;
            case 5: // rainbowCycle
                rainbowCycle(); // rainbowCyle
                break;

            case 6: // blue color chase
                theaterChase(strip.Color(0, 0, 255), 50); // Chase Blue
                break;        
            case 7: // red color chase
                theaterChase(strip.Color(255, 0, 0), 50); // Chase red
                break;  
            case 8: // green color chase
                theaterChase(strip.Color(0, 255, 0), 50); // Chase green
                break;
            case 9: // color chase
                theaterChaseRainbow(50); // Chase rainbow
                break;

           // case 10: // random color wipe
             //   int randomRed = (currentProgramPrioity * 2); //use current priority to set color
             //   int randomGreen = currentProgramPrioity; //use current priority to set color
             //  int randomBlue = (currentProgramPrioity * .75); //use current priority to set color
                 
             //   randomRed = randomRed > 255 ? 256 : randomRed; //only allow upto 256
             //   randomGreen = randomGreen > 255 ? 256 : randomGreen; //only allow upto 256
             //   randomBlue = randomBlue > 255 ? 256 : randomBlue; //only allow upto 256

             //   colorWipe(strip.Color(randomRed, randomGreen, randomBlue)); // random red brightness
             //   break;
                
        }
        
        previousLedProgram = currentLedProgram;
    }
    
    /***********************************************************************/
    // Packet receive
    // Check if there is a new packet available, if there is, and the rssi
    // is above a threshold, decode it and
    // use it as the requested program and priority
    /***********************************************************************/
    const int minRssiThreshold = -30;
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
            
            sprintf(buffer, "Packet len: %d \"%s\" Rssi: %d", len, (char*)packet, lastRssi);
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
                    sprintf(buffer, "Got req: %d %d Rssi: %d", requestedLedProgram, requestedProgramPrioity, lastRssi);
                }
                else {
                    Serial.println("Bad packet");
                }
            } // end if rssi threshold
        } // end if recv packet
        else {
            char buffer[255];
            sprintf(buffer, "Packet receive failed. len: %d", len);
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
    const unsigned long transmitPeriodMs = 10*1000; // 10s
    if (currentMillis - previousTransmitMillis >= transmitPeriodMs){
        previousTransmitMillis = currentMillis;
        char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
        char buffer[255];
        
        // generate a random new program with random priority
        requestedProgramPrioity = (int16_t)random(1, minimumProgramTimeMs / priorityDecrementPeriodMs);
        requestedLedProgram = (uint8_t)random(0, 10); //min inclusive, max exclusive
        sprintf(radiopacket, "%d %d", requestedLedProgram, requestedProgramPrioity);
        
        sprintf(buffer, "Sending prg:%d pri:%d pack:\"%s\" len: %d", requestedLedProgram, requestedProgramPrioity, radiopacket, strlen(radiopacket));
        Serial.println(buffer);

        // Send a message!
        rf69.send((uint8_t*)radiopacket, strlen(radiopacket));
        rf69.waitPacketSent();
    }

    
}



