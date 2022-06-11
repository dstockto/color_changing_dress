#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_NeoPixel.h>

#define LED_COUNT 20
#define LED_PIN 12
// If you want the strip to turn on one second at a time, change ANIMATE_STRIP to 1 below
#define ANIMATE_STRIP 0
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
#define RFM69_CS      8
#define RFM69_INT     7
#define RFM69_RST     4
#define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
// Feather M0 w/Radio
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

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
#define RFM69_INT     9  // "A"
#define RFM69_CS      10  // "B"
#define RFM69_RST     11  // "C"
#define LED           13

#elif defined(ESP32)    // ESP32 feather w/wing
#define RFM69_RST     13   // same as LED
#define RFM69_CS      33   // "B"
#define RFM69_INT     27   // "A"
#define LED           13
#endif

#if defined(ARDUINO_NRF52832_FEATHER)
/* nRF52832 feather w/wing */
#define RFM69_RST     7   // "A"
#define RFM69_CS      11   // "B"
#define RFM69_INT     31   // "C"
#define LED           17
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

//void Blink(byte PIN, byte DELAY_MS, byte loops);
void setStrip(char red, char green, char blue);

void setup() {
    Serial.begin(9600);

    // If you're debugging on the computer, uncomment the line below.
    // If you're running without a computer you do NOT want the line below.
    //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

    strip.begin();
    strip.show();
    strip.setBrightness(255); // Adjust brightness

    pinMode(LED, OUTPUT);
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    Serial.println("Feather RFM69 RX Test!");
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
    uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                     0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
    };
    rf69.setEncryptionKey(key);

    pinMode(LED, OUTPUT);

    Serial.print("RFM69 radio @");
    Serial.print((int) RF69_FREQ);
    Serial.println(" MHz");

    // Set the entire strip to blue - this indicates the setup part is done
    // The default color is set below with the red, green and blue values - it is set to white. This means after
    // the 1500ms timeout (assuming you don't get a message in the first 1.5s) then the dress will turn white. That
    // means you can consider everything ready to go for sure once the dress turns white.
    setStrip(0, 0, 255);
}

// Default starting colors - all 255s = white - The setup phase above ends with the dress being blue
char red = 255;
char green = 255;
char blue = 255;
char white = 0; // probably don't need white unless you have an RGBW strip

void loop() {

    // Wait for 1.5 seconds for a message from the transmitter
    if (rf69.waitAvailableTimeout(1500)) {
        // Should be a message for us now.
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        // The message we're looking for looks like "C<R><G><B>" where C is literally C, and <R>, <G> and <B> are a
        // single byte representing a color value from 0-255. So it will look like C followed by some junk characters
        // most of the time when we print below
        if (rf69.recv(buf, &len)) {
            if (!len) {
                return;
            }
            buf[len] = 0;
            Serial.print("Received [");
            Serial.print(len);
            Serial.print("]: ");
            Serial.println((char *) buf);
            Serial.print("RSSI: ");
            Serial.println(rf69.lastRssi(), DEC);

            red = buf[1];
            green = buf[2];
            blue = buf[3];
            white = buf[4];

            char reply[80];
            // This will show the numerical values of the R, G and B values along with the timestamp of when it was
            // received. This code was originally written for pixies which must be refreshed at least every 2 seconds
            // or they shut off (hence the 1500 ms wait above). The millis() call tells how the time in milliseconds
            // since the microprocessor started. We are printing out what we got to serial, but also sending back
            // a message to the sender indicating what we got. These helped with being sure that what we sent was what
            // we got.
            Serial.printf("%d: Got %c R: %d G: %d B: %d W: %d\n", millis(), buf[0], red, green, blue, white);
            sprintf(reply, "Got %c R: %d G: %d B: %d W: %d\n", buf[0], red, green, blue, white);

            rf69.send((uint8_t *) reply, strlen(reply));
            rf69.waitPacketSent();

            setStrip(red, green, blue);

            Blink(LED, 40, 3); // This is a little onboard LED - if you are transmitting and this board is receiving,
            // you'll see it blink 3 times pretty rapidly on every receive which is nice to have before the LED strip
            // is connected
        } else {
            Serial.println("Receive failed");
        }
    } else {
        // We are setting the color to the strip every 1500ms if we don't get a new color we send what we had last time
        // to the strip. This is helpful if you end up using Pixies (very, very bright and powerful) since they must
        // be refreshed regularly. They will shut off automatically at 2 seconds since the last time they received an
        // instruction. If you're using neopixels, you can remove this block. It doesn't hurt anything, it's just not
        // needed.
        Serial.println("Waited 1500ms for receive, trying to refresh");
        Serial.printf("R: %d G: %d, B: %d\n", red, green, blue);
        setStrip(red, green, blue);
    }
}


void setStrip(char red, char green, char blue) {
    for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color((int) red, (int) green, (int) blue));
#if ANIMATE_STRIP
        // This will make each section light up for a little bit (50ms) before turning on the next section. If it is
        // disabled then the color change will be instantaneous.
        strip.show();
        delay(50);
#endif
    }
    strip.show();
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
    for (byte i = 0; i < loops; i++) {
        digitalWrite(PIN, HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN, LOW);
        delay(DELAY_MS);
    }
}
