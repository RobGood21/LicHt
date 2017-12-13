/*
 Name:		Sketch1.ino
 Created:	11/26/2017 10:28:59 PM
 Author:	gebruiker
*/

#include <FastLED.h>
#define NUM_LEDS 15
#define DATA_PIN 6

unsigned long TimerWrite;
unsigned long TimerSet;

CRGB leds[NUM_LEDS];

unsigned int rood[NUM_LEDS] = { 0x0 };
unsigned int groen[NUM_LEDS] = { 0x0 };
unsigned int blauw[NUM_LEDS] = { 0x0 };



void setup() {
	pinMode(13, OUTPUT);
	FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
	TimerWrite = millis();
	TimerSet = millis();
}
void SetLeds() { //steld de leds in		
	static int i = 0;
	static int c = 0; //kleur keuze
	int kleur;

				 if (digitalRead(13) == HIGH) {
				 digitalWrite(13, LOW);
				 }
				 else {
				 digitalWrite(13, HIGH);
				 }			

		//if (i == 0) {
		//	leds[NUM_LEDS - 1] = 0x000000;
		//}
		//else {
		//	leds[i - 1] = 0x000000;
		//}
		//leds[i] = CRGB::Orange;

//Bepaal wat led I moet doen, als voorbeeld een looplicht
				 switch (c){
				 case 0:
					 leds[i] = 0x010101;
					 break;
				 case 1:
					 leds[i] = 0x330000;
					 break;
				 case 2:
					 leds[i] = 0x003300;
					 break;
				 case 3:
					 leds[i] = 0x000033;
					 break;
				 case 4:
					 leds[i] = 0x225522;
					 break;
				 case 5:
					 leds[i] = 0x660033;
						 break;
				 }

				 
				 //voorgaande led weer uit..
				 if (i == 0) {
				 	leds[NUM_LEDS - 1] = 0x000000;
				 }
				 else {
				 	leds[i - 1] = 0x000000;
				 }



 i++;
 if (i > NUM_LEDS) {
	 i = 0;
	 c++;
	 if (c > 5) c = 0; //kleur restten
 }
}

void SendLeds() { //verstuurt de (nieuw) data naar de ledlijst ieder 10ms
		FastLED.show();
	}

// the loop function runs over and over again until power down or reset
void loop() {

	if (millis() - TimerWrite > 1) {
		TimerWrite = millis();
		SendLeds();
	}

	if (millis() - TimerSet > 300) {
		TimerSet = millis();
		SetLeds();
	}

}

