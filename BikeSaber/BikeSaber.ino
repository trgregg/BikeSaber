///
/// @mainpage	blinkLed
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Travis Gregg
/// @author		marroug
/// @date		December, 12 3:01 PM
/// @version	<#version#>
///
/// @copyright	(c) Travis Gregg, 2018
/// @copyright	Licence
///
/// @see		ReadMe.txt for references
///


/// Largely borrowed from Justin Gregg's BikeSaber project
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
#define ANALOG1            A1
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
#define NUMPIXELS 155
//#define NUMPIXELS 10
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

void xMasUpdateColor(uint8_t Red, uint8_t Green, uint8_t Blue) {
        char buffer[255];
        sprintf(buffer, "Setting %d LEDs to red:%d green:%d blue:%d", NUMPIXELS, Red, Green, Blue);
        Serial.println(buffer);
        for(int i=0; i < (strip.numPixels()); i++){
             strip.setPixelColor(i, strip.Color(Red, Green, Blue));
             }
        strip.show();
        delay(50);
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
    static unsigned long previousLedUpdatedMillis = 0;
    static unsigned long previousTransmitMillis = 0;
    static unsigned long previousColorUpdateMillis = 0;
    const unsigned long transmitPeriodMs = 100; 
    const unsigned long pickNewColorPeriodMs = 60*1000; // 60s 

    // Setup analog input to get manual color selection
    static int analogInput1 = 0;

    // led timeout for when we don't recieve a packet, meaning the braodcast controller is OFF, or we missed 10 packets
    const unsigned long ledUpdatePeriodMs = 1000; // 1s

    // Start broadcasting with LED color Red
    static int sendRed = 200; 
    static int sendGreen = 0; 
    static int sendBlue = 0;
    static int requestedRed = 200; 
    static int requestedGreen = 0; 
    static int requestedBlue = 0;

    // current time info
    unsigned long currentMillis = millis();

    // Slave mode (no transmit)
    const int slaveMode = 0;

    digitalWrite(LED_PIN, isLEDOn);
    isLEDOn = !isLEDOn;
       
    /***********************************************************************/
    // If we haven't received a packet and updated the LEDs in a while, turn off the LEDs
    /***********************************************************************/
    if (currentMillis - previousLedUpdatedMillis > ledUpdatePeriodMs){
        // update the previous time record
    

        // Since we didn't get any packets, setting LEDs OFF
        int Red = 0, Green = 0, Blue = 0;
        
        char buffer[255];

        sprintf(buffer, "Turning LED OFF: %d %d %d", Red, Green, Blue);
        Serial.println((char*)buffer);
        
        xMasUpdateColor(Red, Green, Blue);
        previousLedUpdatedMillis = currentMillis;

    }
    
    /***********************************************************************/
    // Packet receive
    // Check if there is a new packet available, if there is, and the rssi
    // is above a threshold, decode it and
    // use it as the requested color, update LEDs, and update previousLedUpdateMillis
    /***********************************************************************/
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
            
            sprintf(buffer, "Packet len: %d \"%s\" Rssi: %d", len, (char*)packet, lastRssi);
            Serial.println(buffer);
            
            // check the packet's rssi
            if(lastRssi > minRssiThreshold){
                int tempRed = 0, tempGreen = 0, tempBlue = 0;
                int numFound = 0;
                
                numFound =  sscanf((char*)packet, "%d %d %d", &tempRed, &tempGreen, &tempBlue);
                
                // if we got two items parsed out of the packet, use them for req
                if (numFound == 3){
                    requestedRed = tempRed;
                    requestedGreen = tempGreen;
                    requestedBlue = tempBlue;
                    
                    sprintf(buffer, "Got req: %d %d %d Rssi: %d", requestedRed, requestedGreen, requestedBlue, lastRssi);
                    // Update the LEDs based on what we received
                   xMasUpdateColor(requestedRed, requestedGreen, requestedBlue);
                   previousLedUpdatedMillis = currentMillis;
                   
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
    // Send out an RGB color every 100ms
    // If it has been a minute since we changed color, change the color
    /***********************************************************************/
    /***********************************************************************/
    // Comment out this transmit for slave units for Xmas Lights
    /***********************************************************************/
    
    if (((currentMillis - previousTransmitMillis >= transmitPeriodMs) || (previousTransmitMillis == 0)) && slaveMode == 0) {
        previousTransmitMillis = currentMillis;
        char radiopacket[RH_RF69_MAX_MESSAGE_LEN];
        char buffer[255];

        // Read analog input which should return 0-1024
        analogInput1 = analogRead(ANALOG1);
        int knobPosition = analogInput1 / 4 - 1; // Divide by 4 to get range of 1024 -> 256

        // if the analog input isn't turned all the way down, get the value and tranlate it to a number to look up RGB on a color wheel
        if (knobPosition > 10) {
            char buffer[255];
            sprintf(buffer, "Knob position: %d", knobPosition);
            Serial.println(buffer);

            // Make a RGB color based on KnobPositon
            // Largely stolen from "Wheel" color picker function
            // should return RGB
//            knobPosition = 255 - knobPosition;
            if (knobPosition < 85) {
                sendRed = 255 - knobPosition * 3;
                sendGreen = 0;
                sendBlue = knobPosition * 3;
            }
            else if (knobPosition < 170) {
                knobPosition -= 85;
                sendRed = 0;
                sendGreen = knobPosition * 3;
                sendBlue = 255 - knobPosition * 3;
            }
            else {
                knobPosition -= 170;
                sendRed = knobPosition * 3;
                sendGreen = 255 - knobPosition * 3;
                sendBlue = 0;
            }
        }

        else {
        
            // Make a selection for the color we want to send if it's been a minute since we last changed
            if ((currentMillis - previousColorUpdateMillis >= pickNewColorPeriodMs) || (previousColorUpdateMillis == 0 )) {
                Serial.println("Picking a new random color");
                previousColorUpdateMillis = currentMillis;
               
                // Pick a RGB color to send to listeners
                int pickColor = (uint8_t)random(1, 5); //min inclusive, max exclusive
                sprintf(buffer, "Choosing Color Case: %d", pickColor);
                Serial.println(buffer);
                
                switch(pickColor){
                    default:
                        Serial.println("Defaulting to OFF");
                        // fall through to use 0 as default
                    // Xmas lights
                    case 0: // black/off color wipe
                        sendRed = 0, sendGreen = 0, sendBlue = 0; // OFF
                        break;
                    case 1: // xMas Red mode
                        sendRed = 200, sendGreen = 0, sendBlue = 0;
                        break;
                    case 2: // xMas Green mode
                       sendRed = 25, sendGreen = 250, sendBlue = 0;
                       break;
                    case 3: // xMas White mode
                        sendRed = 200, sendGreen = 100, sendBlue = 200;
                        break;
                    case 4: // xMas Blue mode
                        sendRed = 100, sendGreen = 0, sendBlue = 200;
                        break;
                } // end switch
            } // end auto random color picker
        } // end if-else for color update
        
        //update the local LEDs if they exist
        xMasUpdateColor(sendRed, sendGreen, sendBlue);
        previousLedUpdatedMillis = currentMillis;

        // Broadcast the color we want
        sprintf(radiopacket, "%d %d %d", sendRed, sendGreen, sendBlue);
        
        sprintf(buffer, "Sending red:%d green:%d blue:%d", sendRed, sendGreen, sendBlue);
        Serial.println(buffer);

        // Send a message!
        rf69.send((uint8_t*)radiopacket, strlen(radiopacket));
        rf69.waitPacketSent();
    } // end transmit
    
} // end loop



