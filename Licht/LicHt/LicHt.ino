/*
 Name:		LicHt.ino
 Created:	12/23/2017 10:57:27 AM
 Author:	Rob Antonisse
 Version: 1.01

 LicHT is a project for full automated lighting of the model railroad, including day and night cyclus.
 Based on WS2811 adressable led control. 

 In void setup_led is te lezen welke led op welk programma reageert en hoe dit aan te passen. 
*/

#include <FastLED.h>
#include <eeprom.h>


//#define LQ 64 //aantal leds in ledstring
#define LPdl 8 //pin waar de daglicht ledstring op komt
#define LPvl 7 //pin voor verlichting string
#define LPev 6 //Pin voor events ledstrip

#define tday 15 //tday how long is a modeltimeday in minute 24 is good value lager dan 10 werkt het geheel niet goed


CRGB led_dl[64];
CRGB led_vl[64];
CRGB led_ev[64];

byte led_dlap[64];//daglicht assign program
byte led_vlap[64]; //vlap=verlichting assign program
byte led_evap[64];//evap=event assign program

 //Temp declaration during design, debugging

//Declaraties
int COM_DCCAdres=64;
byte COM_reg; //bit0 test PRG active(true)
byte PRG_reg[64]; //bit0 active(true) bit1=initialised (true) bit7-bit5 exclusive for program
byte PRG_min[64]; //Time next active minute
byte PRG_hr[64]; //Time next actice hour


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
	setup_leds();
	Opening();
}
void setup_leds() {
	/*
	assigns a program outpu number to a led, default is preprogrammed. 
	Programs can have multiple outputs
	Max outputs 254
	255 is reserved if eeprom read gives 255 led, uses default preset value
	Define preset value in void led_preset
	default value can be changed by CV 10 channel 1, 11 channel 2, 12 channel 3, 13 channel 4 
	Use correct decoder adres and choose correct CV. Value 0-255 defines program output number
	See CV tabel in manual. 
	Setting cv value to 255 will set default preprogrammed value.
	Setting cv 3 of main decoder adres resets to default values.
	Make administration for the changes, 
	*/
	//Programs VL:          program number		program outputs			assigned leds
	//PRG_traffic			12					1,2						vl0; vl1
	led_vlap[0] = 1;
	led_vlap[1] = 2;

	//programs DL
	//PRG_Flashlight			11						ev2; ev6

	//check EEprom for modyfied entry
	for (int i = 0; i < 64; i++) {
		if (EEPROM.read(i) < 0xFF) led_dlap[i] = EEPROM.read(i);
	}
	for (int i = 64; i < 128; i++) {
		if (EEPROM.read(i) < 0xFF) led_vlap[i-64] = EEPROM.read(i);
	}
	for (int i = 128; i < 192; i++) {
		if (EEPROM.read(i) < 0xFF) led_evap[i - 128] = EEPROM.read(i);
	}

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
		if (mt_min > 60) {
			mt_min = 0;
			mt_hr ++;
			if (mt_hr > 24) {
				mt_hr = 0;
			}
		}
	}
}
void COM_ProgramAssign() {
	//plays, assigns programs only 1 every cycle 
	//first 64 active programs 1by1 then only 1 not active programs, then 64 active and so on. 
	static unsigned pa; //program active
	static unsigned pna;//program not active
	if (bitRead(COM_reg, 0) == true) { //find active program
		if (bitRead(PRG_reg[pa], 0) == true) COM_ps(pa);
		pa++;
		if (pa > 64) {
			pa = 0;
			COM_reg &= ~(1 << 0); //reset bit 0, next cycle not active
		}
	}
	else { //find not-active program
		if ((PRG_hr[pna] == mt_hr)& (PRG_min[pna] == mt_min))COM_ps(pna);
		COM_reg |= (1 << 0); //next cycle active
		pna++;
		if (pna > 64)pna = 0;
	}
}
void COM_ps(int pn) { //ps=program switch
	//total, max 64 programs
	//1-10 daylight and weather 11/30 lighting houses and streetlights  31 > no idea yet
	switch (pn) {
	case 11:
		PRG_flashlight(pn);
		break;
	case 12:
		PRG_traffic(pn);
		break;

	default:
		//all not defined programs// do nothing
		break;
	}
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
	int dec;
	if (adres >= VL_adresmin & adres <= VL_adresmax) {
		if (type == false) {//switch
			if (port == true) {
				PORTB |= (1 << 5);
				led_vl[adres - VL_adresmin] = 0xAAAAAA; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
			}
			else {
				PORTB &= ~(1 << 5);
				led_vl[adres - VL_adresmin] = 0x000000;
			}
			FastLED.show();
		}
	}
	else {//CV
		dec = 4*(decoder - (COM_DCCAdres)-17); //geeft led volgorde van led op channel 00 van deze (sub)decoder
		
		switch (cv) { //adres is altijd hier het adres van de decoder. Niet van het channel
		case 10:
			dec = dec + 0;
			led_vlap[dec] = value;
			EEPROM.write(adres - dec + 64, value);
			break;
		case 11:
			dec = dec + 1;
			led_vlap[dec] = value;
			EEPROM.write(adres - dec + 64, value);
			break;
		case 12:
			dec = dec + 2;
			led_vlap[dec] = value;
			EEPROM.write(adres - dec + 64, value);
			break;
		case 13:
			dec = dec + 3;
			led_vlap[dec] = value;
			EEPROM.write(adres - dec + 64, value);
			break;
		}
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

void LED_set(byte group,byte output, byte r,byte g,byte b) { 
	//group welke ledstring 0=dl 1=vl 2=ev 
	//output is output nummer van het programma
	//color ? eerst alleen even als hex
	//kleur aanduiding hier keuzes mogelijk maken???? 
	//sets led value from programs
	//fastled.show command hier geven?????

	//64 leds in een string kan veel meer zijn toch?

	for (byte i=0; i < 65; i++) { //check all leds in this group
		switch (group) {
		case 0:
			if (led_dlap[i] == output)led_dl[i] = CRGB(r,g,b);
			break;
		case 1:
			if (led_vlap[i] == output)led_vl[i] = CRGB(r, g, b);
			break;
		case 2:
			if (led_evap[i] == output)led_ev[i] = CRGB(r, g, b);
			break;
		}
	}
}
void PRG_flashlight(int pn) {//
	/*
voorbeeld van een PRG_ in LicHt
knipperlicht op twee leds in vl (verlichting)
programma geeft iedere 10 minuten na het uur, 5 minuten een knipperlicht. 

declaraties voor dit PRG_
PRG_reg[n]gebruik:
bit7: status knipperlict welk helft is aan
bit6: 
bit5:
bit4:

Dit programma gebruikt 2leds

doorloop verkort
-declaraties van variabelen en gebruik van de bits in PRG_reg[n] voor dit programma
-Instellen voor als arduino is gestart.
-eerste aanroep vanuit COM_ps, programma actief maken, voorwaarde voor stoppen instellen
-programma uitvoeren
-voorwaarde voor stoppen bereikt, programma stoppen en voorwaarde voor opnieuw starten instellen.

*/
	static unsigned int eindtijd; //wanneer stoppen
	static int ft = 125; //flashtime 100ms
	static unsigned long f = 0;

	//******************initialiseren 
	//bij eerste doorloop bit 1 in PRG_reg[n] is dan false. Geeft aan dat PRG_ nog niet is ingesteld
	//Moet ieder programma mee beginnen.
	//gebruikte leds bepalen.
	
	if (bitRead(PRG_reg[pn], 1) == false) {//init
		PRG_reg[pn] |= (1 << 1);//set initialiseer bit
		PRG_hr[pn] = 0; //zet uur waar programma actief moet worden
		PRG_min[pn] = 10;//zet minuut waar program in actief moet worden.
	}
	else { //no init needed

		//**************program actief
		//Dus instellingen bij eerste doorloop
		//Program is nu aangeroepen vanuit COM_ps
		//als PTG_reg[n] bit 0 false is dan is dit de eerste keer in deze actieve cyclus
		//nu dus actie in gang zetten
		//diverse nog in te stellen parameters kunnen gebruikt worden om het proces te stoppen.
		//stoppen gebeurt door dit bit weer false te maken.
		//De ingestelde tijd hr en min zullen bij bereiken deze PRG_ weer aanroepen. 

		if (bitRead(PRG_reg[pn], 0) == false) { //nog niet actief, eerste doorloop
			PRG_reg[pn] |= (1 << 0); //maak aktief
			eindtijd = mt_min + 5; //5 modeltijd minuten uitvoeren	
			PRG_hr[pn] = mt_hr + 1;
			if (PRG_hr[pn] > 24) PRG_hr[pn] = 0;

		}
		else {//actief programma
			//eerst kijken of het weer moet stoppen
			if (mt_min > eindtijd) {//stoppen, alles kan het stoppen veroorzaken. in dit program gewoon een 5 tal verlopen modeltijd minuten
				//alles restten voor de volgende actieve periode
				//ft = ft/2; //knipper periode in milliseconde
				f = 0;
				led_ev[2] = 0x000000; //black leds
				led_ev[6] = 0x000000;
				FastLED.show();

				PRG_reg[pn] &= ~(1 << 0); //set non actif
				//nieuwe starttjd instellen, minuten hoeven hier niet

			}
			else {

				//*********hier het actieve deel van het programma
				//in dit voorbeeld geval een knipperlicht.
				if (millis() - f > ft) {
					f = millis();
					PRG_reg[pn] ^= (1 << 7); //gebruik bit 7 van register
					if (bitRead(PRG_reg[pn], 7) == true) {
						led_ev[2] = 0xAA0000;
						led_ev[6] = 0x000000;
					}
					else {
						led_ev[2] = 0x000000;
						led_ev[6] = 0xAA0000;
					}
					FastLED.show();
				}
			}
		} 
	}//init
}
void PRG_traffic(int pn) {
//verkeerslicht, starts and runs forever...
	//outputs  1=1 2=2
	

	static unsigned long tijd;
	static unsigned int periode=500;
	static byte fase=0;
	static byte WieMag;
	static byte out;

	if (bitRead(PRG_reg[pn], 1) == false) { 
		PRG_reg[pn] |= (1 << 1); //no init 
		PRG_reg[pn] |= (1 << 0); //actif
		//no time needed, always active
	}
	else {
		if (millis() - tijd > periode) {
			tijd = millis();

			switch (fase) {
			case 0: //begin
				fase = 1;
				WieMag = 1;
				out = 2;
				periode = 5000;
				LED_set(1, 1, 200,0,0);
				LED_set(1, 2, 200,0,0);
				break;
			case 1:
				periode = 100;
				fase = 2;

				switch (out) {
				case 1:					
					out = 2;
					break;
				case 2:
					out = 1;
					break;
				}

			case 2:
				periode = 12000;
				LED_set(1, out, 0,200,0);
				fase = 3;
				break;

			case 3: //OW oranje
				periode = 4000;
				LED_set(1, out, 0,0,150);
				fase = 4;
				break;
			case 4:
				//all red
				periode = 5000;
				LED_set(1, 1, 200,0,0);
				LED_set(1, 2, 200,0,0);
				fase = 1;
				break;
			}
			FastLED.show();
		}
	}
}


void loop() {
	COM_Clk();
	COM_ProgramAssign();
	DEK_DCCh();
}
