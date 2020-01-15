#include <vector>
#include <Arduino.h>
#include "FastLED.h"


using namespace std;


const int nLed       = 12;
const int dataPin    = 23;
const int relay[]    = {5, 17, 16, 4};

const int packetSize = nLed * 3 + 1;

std::vector<unsigned char> packet(packetSize);
int i = 0;

CRGB leds[nLed];


void setup()
{
    Serial.begin(115200 * 8);

    pinMode(relay[0], OUTPUT);
    pinMode(relay[1], OUTPUT);
    pinMode(relay[2], OUTPUT);
    pinMode(relay[3], OUTPUT);

    gpio_set_drive_capability(GPIO_NUM_23, GPIO_DRIVE_CAP_2);
    FastLED.addLeds<NEOPIXEL, dataPin>(leds, nLed);
    FastLED.show();
}

void loop()
{
    int c = Serial.read();

    if (c >= 0)
    {
        packet[i++] = c;

        if (i >= packetSize)
        {
            i = 0;
            
            digitalWrite(relay[0],  (packet[0])       & 1);
            digitalWrite(relay[1], ((packet[0]) >> 1) & 1);
            digitalWrite(relay[2], ((packet[0]) >> 2) & 1);
            digitalWrite(relay[3], ((packet[0]) >> 3) & 1);

            int p = 1;
            for (int j = 0; j < 12; j++)
            {
                int r = packet[p];
                int g = packet[p + 1];
                int b = packet[p + 2];

                leds[j] = (r << 16) + (g << 8) + b;
               
                p += 3;
            }

            FastLED.show();
        }
    }
}