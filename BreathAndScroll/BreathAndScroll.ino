#include <FastLED.h>

#define LEDPin 13
#define numLEDs 35

CRGB leds[numLEDs];

void setup() {
  FastLED.addLeds<WS2812, LEDPin, GRB>(leds, numLEDs);

}

void loop() {
  unsigned int rgbColour[3];

  rgbColour[0] = 255;
  rgbColour[1] = 0;
  rgbColour[2] = 0;

  // Choose the colours to increment and decrement.
  for (int decColour = 0; decColour < 3; decColour += 1) {
    int incColour = decColour == 2 ? 0 : decColour + 1;

    // cross-fade the two colours.
    for(int i = 0; i < 255; i += 15) {
      rgbColour[decColour] -= 15;
      rgbColour[incColour] += 15;

      for (int j = 3; j < 29; j++) {
        leds[j+1] = CRGB (rgbColour[0], rgbColour[1], rgbColour[2]);
        FastLED.show();
        leds[j] = CRGB(0,0,0);
        FastLED.show();
        delay(50);
      }
      for (int m = 29; m >= 3; m--) {
        leds[m] = CRGB (rgbColour[0], rgbColour[1], rgbColour[2]);
        FastLED.show();
        leds[m+1] = CRGB(0,0,0);
        FastLED.show();
        delay(50);
      }
    }
  }
}
