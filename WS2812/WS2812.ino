/*
Programma voor aansturen van een WS2812 Led array

*/

#include <FastLED.h>
#define NUM_LEDS 8
#define DATA_PIN 6


CRGB leds[NUM_LEDS];


void setup() { FastLED.addLeds<NEOPIXEL, 6>(leds, NUM_LEDS); }
void loop() {
	leds[0] = CRGB::White; FastLED.show(); delay(30);
	leds[0] = CRGB::Black; FastLED.show(); delay(30);
}