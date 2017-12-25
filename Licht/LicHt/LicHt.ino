/*
 Name:		LicHt.ino
 Created:	12/23/2017 10:57:27 AM
 Author:	Rob Antonisse
 Version: 1.01

 LicHT is a project for full automated lighting of the model railroad, including day and night cyclus.
 Based on WS2811 adressable led control. 

*/

#include <FastLED.h>

//#define LQ 64 //aantal leds in ledstring
#define LPdl 8 //pin waar de daglicht ledstring op komt
#define LPvl 7 //pin voor verlichting string
#define LPev 6 //Pin voor events ledstrip

#define tday 24 //tday how long is a modeltimeday in minutes


CRGB led_dl[64];
CRGB led_vl[64];
CRGB led_ev[64];

//Temp declaration during design, debugging

//Declaraties
int COM_DCCAdres=64;

unsigned long Clk;
unsigned int mt; //modeltime minute
unsigned int mt_min; //modeltimeclock minutes 
unsigned int mt_hr; //modeltimeclock hours

//Zorg dat bij programmeren altijd een BASIS adres wordt genomen. dus 1=(1-4)2=(5-8) 3=(9-12) 4=(13-16) 5 6 enz 

//basic adres, adres Daylight decoder DL +1 
//VL (verlichting) decoder 16 adresses higher
//EV (events) decoder 16adresses higher

int DL_adresmin;
int DL_adresmax;
int VL_adresmin;
int VL_adresmax;
int EV_adresmin;
int EV_adresmax;


volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
boolean DEK_Monitor = false; //shows DCC commands as bytes
byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[12]; //registerbyte for 12 command buffers
					 //bit7= free (false) bit0= CV(true)
byte DEK_Buf0[12];
byte DEK_Buf1[12];
byte DEK_Buf2[12];
byte DEK_Buf3[12];
byte DEK_Buf4[12];
byte DEK_Buf5[12];

void setup() {
	Serial.begin(9600);
	//test mode
	DDRB |= (1 << 5);	//pin13
	DDRB |= (1 << 4);  //Pin12 als output
	Clk = millis();

	//Fastled part
	FastLED.addLeds<NEOPIXEL, LPdl>(led_dl, 64); //create strip of 64 leds on pin 8 'Daglicht'
	FastLED.addLeds<NEOPIXEL, LPvl>(led_vl, 64);//create strip of 64 leds on pin7 'verlichting'
	FastLED.addLeds<NEOPIXEL, LPev>(led_ev, 64);//create strip of 64 leds on pin6 'Events'
	
	DL_adresmin = (COM_DCCAdres * 4) + 1; //no mistake, COM_DCCadres for decoder = 1 lower, COM_DCCadres+1 1th led adres.
	DL_adresmax = DL_adresmin + 63;
	VL_adresmin = DL_adresmax + 1;
	VL_adresmax = VL_adresmin + 63;
	EV_adresmin = VL_adresmax + 1;
	EV_adresmax = EV_adresmin + 63;
	

	//DeKoder part, interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1

	//program part
	mt = (tday * 1000) / 24;

	Opening();
}

void Opening() {
	Serial.println("");
	Serial.println("");
	Serial.println("Welkom bij LicHT");
	Serial.println("Een wisselmotor.nl project");
	Serial.println("-------------------");
	Serial.println("Instellingen:");
	Serial.print("Decoder adres: ");
	Serial.print(COM_DCCAdres);
	Serial.print(" (");
	Serial.print(((COM_DCCAdres-1)*4)+1);
	Serial.print("-");
	Serial.print(((COM_DCCAdres - 1) * 4) + 4);
	Serial.println(")");
	Serial.print("adressen daglicht leds van ");
	Serial.print(DL_adresmin);
	Serial.print(" tot en met ");
	Serial.println(DL_adresmax);
	Serial.print("adressen verlichting leds van ");
	Serial.print(VL_adresmin);
	Serial.print(" tot en met ");
	Serial.println(VL_adresmax);
	Serial.print("adressen gebeurtenissen leds van ");
	Serial.print(EV_adresmin);
	Serial.print(" tot en met ");
	Serial.println(EV_adresmax);
	Serial.println("");
	Serial.print("Dag, 24uur in modeltijd duurt ");
	Serial.print(tday);
	Serial.println(" minuten.");

}
ISR(INT0_vect) { //syntax voor een ISR
				 //isr van PIN2
				 //DEK_Reg fase van bit ontvangst
				 //bit 0= bitpart ok (1) of failed(0)
				 //bit1= truepart 
				 //bit2=falsepart
				 //bit3= received bit true =true false =false
				 //bit4=restart, begin, failed as true

	cli();
	DEK_duur = (micros() - DEK_Tperiode);
	DEK_Tperiode = micros();
	if (DEK_duur > 50) {
		if (DEK_duur < 62) {
			DEK_Reg |= (1 << 0); //bitSet(DekReg, 0);

			if (bitRead(DEK_Reg, 1) == false) {
				DEK_Reg &= ~(1 << 2); //bitClear(DekReg, 2);
				DEK_Reg |= (1 << 1);
			}
			else { //received full true bit

				DEK_Reg |= (1 << 3);
				DEK_BitRX();
				DEK_Reg &= ~(1 << 2);//bitClear(DekReg, 2);
				DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
			}
		}
		else {
			if (DEK_duur > 106) {

				if (DEK_duur < 124) { //preferred 118 6us extra space in false bit
					DEK_Reg |= (1 << 0); //bitSet(DekReg, 0);
					if (bitRead(DEK_Reg, 2) == false) {
						DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
						DEK_Reg |= (1 << 2);  //bitSet(DekReg, 2);
					}
					else { //received full false bit
						DEK_Reg &= ~(1 << 3); //bitClear(DekReg, 3);
						DEK_BitRX();
						DEK_Reg &= ~(1 << 2);//bitClear(DekReg, 2);
						DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
					}
				}
			}
		}
	}
	sei();
}
void DEK_begin() {//runs when bit is corrupted, or command not correct
				  //lesscount++;
	DEK_countPA = 0;
	DEK_Reg = 0;
	DEK_Status = 0;
	for (int i = 0; i < 6; i++) {
		DEK_byteRX[i] = 0; //reset receive array
	}
}
void DEK_BufCom(boolean CV) { //create command in Buffer
	byte i = 0;
	while (i < 12) {

		if (bitRead(DEK_BufReg[i], 7) == false) {
			DEK_BufReg[i] = 0; //clear found buffer


			DEK_Buf0[i] = DEK_byteRX[0];
			DEK_Buf1[i] = DEK_byteRX[1];
			DEK_Buf2[i] = DEK_byteRX[2];

			if (CV == true) {
				DEK_BufReg[i] |= (1 << 0); //set for CV
				DEK_Buf3[i] = DEK_byteRX[3];
				DEK_Buf4[i] = DEK_byteRX[4];
				DEK_Buf5[i] = DEK_byteRX[5];
			}
			else {

				DEK_Buf3[i] = 0;
				DEK_Buf4[i] = 0;
				DEK_Buf5[i] = 0;
			}
			DEK_BufReg[i] |= (1 << 7); //claim buffer
			i = 15;
		}
		i++;
	} //close for loop
} //close void
void DEK_BitRX() { //new version
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0;
	static byte n = 0;
	DEK_Reg |= (1 << 4);//resets and starts process if not reset in this void
	switch (DEK_Status) {
		//*****************************
	case 0: //Waiting for preample 
		if (bitRead(DEK_Reg, 3) == true) {
			DEK_countPA++;
			if (DEK_countPA >12) {
				DEK_Status = 1;
				countbit = 0;
				countbyte = 0;
			}
			bitClear(DEK_Reg, 4);
		}
		break;
		//*************************
	case 1: //Waiting for false startbit
		if (bitRead(DEK_Reg, 3) == false) { //startbit receive
			DEK_countPA = 0;
			DEK_Status = 2;
		}
		//if Dekreg bit 3= true no action needed.
		bitClear(DEK_Reg, 4); //correct, so resume process
		break;
		//*************************
	case 2: //receiving data
		if (bitRead(DEK_Reg, 3) == true) DEK_byteRX[countbyte] |= (1 << (7 - countbit));
		countbit++;
		if (countbit == 8) {
			countbit = 0;
			DEK_Status = 3;
			countbyte++;
		}
		bitClear(DEK_Reg, 4); //correct, so resume process
		break;
		//*************************
	case 3: //waiting for separating or end bit
		if (bitRead(DEK_Reg, 3) == false) { //false bit
			DEK_Status = 2; //next byte
			if ((bitRead(DEK_byteRX[0], 6) == false) & (bitRead(DEK_byteRX[0], 7) == true))bitClear(DEK_Reg, 4); //correct, so resume process	
		}
		else { //true bit, end bit, only 3 byte and 6 byte commands handled by this dekoder
			switch (countbyte) {
			case 3: //Basic Accessory Decoder Packet received
					//check error byte
				if (DEK_byteRX[2] = DEK_byteRX[0] ^ DEK_byteRX[1])DEK_BufCom(false);
				break; //6
			case 6: ///Accessory decoder configuration variable Access Instruction received (CV)
					//in case of CV, handle only write command
				if (bitRead(DEK_byteRX[2], 3) == true && (bitRead(DEK_byteRX[2], 2) == true)) {
					//check errorbyte and make command
					if (DEK_byteRX[5] = DEK_byteRX[0] ^ DEK_byteRX[1] ^ DEK_byteRX[2] ^ DEK_byteRX[3] ^ DEK_byteRX[4])DEK_BufCom(true);
				}
				break;
			} //close switch bytecount
		}//close bittype
		break;
		//***************************************
	} //switch dekstatus
	if (bitRead(DEK_Reg, 4) == true)DEK_begin();
}
void DEK_DCCh() { //handles incoming DCC commands, called from loop()
	static byte n = 0; //one buffer each passing
	byte temp;
	int decoder;
	int channel = 1;
	int adres;
	boolean port = false;
	boolean onoff = false;
	int cv;
	int value;

	//translate command
	if (bitRead(DEK_BufReg[n], 7) == true) {
		decoder = DEK_Buf0[n] - 128;
		if (bitRead(DEK_Buf1[n], 6) == false)decoder = decoder + 256;
		if (bitRead(DEK_Buf1[n], 5) == false)decoder = decoder + 128;
		if (bitRead(DEK_Buf1[n], 4) == false)decoder = decoder + 64;
		//channel
		if (bitRead(DEK_Buf1[n], 1) == true) channel = channel + 1;
		if (bitRead(DEK_Buf1[n], 2) == true) channel = channel + 2;
		//port
		if (bitRead(DEK_Buf1[n], 0) == true)port = true;
		//onoff
		if (bitRead(DEK_Buf1[n], 3) == true)onoff = true;
		//CV
		if (bitRead(DEK_BufReg[n], 0) == true) {
			cv = DEK_Buf3[n];
			if (bitRead(DEK_Buf2[n], 0) == true)cv = cv + 256;
			if (bitRead(DEK_Buf2[n], 1) == true)cv = cv + 512;
			cv++;
			value = DEK_Buf4[n];
		}
		else {
			cv = 0;
			value = 0;
		}
		COM_exe(bitRead(DEK_BufReg[n], 0), decoder, channel, port, onoff, cv, value);


		//Show Monitor (bytes)
		if (DEK_Monitor == true) {
			Serial.print("buffer= ");
			Serial.print(n);
			Serial.print("  value:  ");
			Serial.print(bitRead(DEK_BufReg[n], 7));
			Serial.print(bitRead(DEK_BufReg[n], 6));
			Serial.print(bitRead(DEK_BufReg[n], 5));
			Serial.print(bitRead(DEK_BufReg[n], 4));
			Serial.print(bitRead(DEK_BufReg[n], 3));
			Serial.print(bitRead(DEK_BufReg[n], 2));
			Serial.print(bitRead(DEK_BufReg[n], 1));
			Serial.print(bitRead(DEK_BufReg[n], 0));
			Serial.println("");

			temp = DEK_Buf0[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf1[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf2[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf3[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf4[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");


			temp = DEK_Buf5[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");
			Serial.println("------");

		}
		//clear buffer
		DEK_BufReg[n] = 0;
		DEK_Buf0[n] = 0;
		DEK_Buf1[n] = 0;
		DEK_Buf2[n] = 0;
		DEK_Buf3[n] = 0;
		DEK_Buf4[n] = 0;
		DEK_Buf5[n] = 0;
	}
	n++;
	if (n > 12)n = 0;
}
void COM_exe(boolean type, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//type=CV(true) or switch(false)
	//decoder basic adres of decoder 
	//channel assigned one of the 4 channels of the decoder (1-4)
	//Port which port R or L
	//onoff bit3 port on or port off
	//cv cvnumber
	//cv value
	int adres;
	adres = ((decoder - 1) * 4) + channel;
	//Applications 
	//APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
	APP_DL(type, adres, decoder, channel, port, onoff, cv, value);
	APP_VL(type, adres, decoder, channel, port, onoff, cv, value);
	APP_EV(type, adres, decoder, channel, port, onoff, cv, value);
}
void COM_Clk() {
	
	if (millis() - Clk > mt) { //1 minute in modelrailroad time
			//modelroad time. 1 day standard 24 minutes. (can be updated by CV, DCC or calculation faster of slower)
			//minium timing is an hour modelroad time, faster events will be done on real time
		Clk = millis();
		mt_min ++;
		PINB |=(1 << 5); // toggle led on pin 13
		if (mt_min == 60) {
			mt_min = 0;
			mt_hr ++;
			if (mt_hr == 24) {
				mt_hr = 0;
			}
		}
	}
}
void COM_ProgramAssign() {
	//plays, assigns programs only 1 every cycle
	static unsigned cpa=0;

	switch (cpa) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		PRG_flashlight();
		break;
	case 3:
		break;
	case 4:
		break;
		//up to max programs 64?
	}
	cpa++;
	if (cpa > 64) cpa = 0; //start over next cycle
}

void APP_Monitor(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//application for DCC monitor
	if (type == true) {
		Serial.print("CV:   , ");
	}
	else {
		Serial.print("Switch, ");
	}
	Serial.print("Adres: ");
	Serial.print(adres);
	Serial.print("(");
	Serial.print(decoder);
	Serial.print("-");
	Serial.print(channel);
	Serial.print("), ");
	//cv
	if (type == true) {
		Serial.print("CV: ");
		Serial.print(cv);
		Serial.print(", waarde: ");
		Serial.print(value);
	}
	else {
		if (port == true) {
			Serial.print("A<, ");
		}
		else {
			Serial.print("R>, ");
		}
		if (onoff == true) {
			Serial.print("On.");
		}
		else {
			Serial.print("Off");
		}
	}
	Serial.println("");
}
void APP_DL(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
//Daglicht 64leds
	if (adres >= DL_adresmin & adres <= DL_adresmax) {
		if (port == true) {
			PORTB |= (1 << 4);
			led_dl[adres-DL_adresmin]= 0x000033; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
		}
		else {
			PORTB &= ~(1 << 4);
			led_dl[adres - DL_adresmin] = 0x000000;			
		}
		FastLED.show();	
	}
}
void APP_VL(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//Verlichting 64leds
	if (adres >= VL_adresmin & adres <= VL_adresmax) {
		if (port == true) {
			PORTB |= (1 << 5);
			led_vl[adres - VL_adresmin] = 0x330000; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
		}
		else {
			PORTB &= ~(1 << 5);
			led_vl[adres - VL_adresmin] = 0x000000;
		}
		FastLED.show();
	}
}
void APP_EV(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//Events 64leds
	if (adres >= EV_adresmin & adres <= EV_adresmax) {
		if (port == true) {
			PORTB |= (1 << 5);
			led_ev[adres - EV_adresmin] = 0x003300; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
		}
		else {
			PORTB &= ~(1 << 5);
			led_ev[adres - EV_adresmin] = 0x000000;
		}
		FastLED.show();
	}
}

void PRG_flashlight() {
//PINB |= (1 << 4);
	//led 1 is in ev line led 1 led2 in ev lin led5
	//test program voor een knipperlicht
	static int ft = 100; //flashtime 100ms
	static unsigned long f=0;
	static byte lr = 0;
	if ((mt_min > 0) & (mt_min < 10)) {
		if (millis() - f > ft) {			
			f = millis();
			lr ^= (1 << 0);
			if (bitRead(lr, 0) == true) {
				led_ev[1] = 0xAA0000;
				led_ev[5] = 0x000000;
			}
			else {
				led_ev[1] = 0x000000;
				led_ev[5] = 0xAA0000;
			}
			FastLED.show();
		}

		}
		if (mt_min == 10) {
			led_ev[0] = 0x000000;
			led_ev[5] = 0x000000;
			FastLED.show();
	}
}

void loop() {
	COM_Clk();
	COM_ProgramAssign();
	DEK_DCCh();
}
