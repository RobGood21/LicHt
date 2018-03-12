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

#define ledtype WS2812

//variables for matrix daglicht, assignable by CV
//nu instelling voor demo
byte led_OW=8; //oost-west aantal leds
byte led_NZ=30; //noord-zuid aantal leds

CRGB led_dl[240];
CRGB led_vl[32];
CRGB led_ev[16];

byte led_vlap[32]; //vlap=verlichting assign program, assign a program to a led
byte led_evap[16];//evap=event assign program

//Declaraties
int COM_DCCAdres=64;
byte COM_reg; 
//bit0 test PRG active(true)
//bit1 ledstrips direction N>Z>N>Z>N>Z>N enz (true) or N>Z N>Z N>Z N>Z enz (false, standard)

//bit3 day or night (also manual) false=day, true = night
//bit4 sunset and sunrise without effects true, false = with effects


byte PRG_reg[32]; 
//bit0 active(true) 
//bit1=initialised (true) gebruik in traffic en flashlight, voor initiele start
//bit2=Time switched true = yes false=no
//bit7-bit4 exclusive for program

byte SW_reg; //register booleans for the switch states
/*
bit0 = status switch A0  (dag nacht)
bit1=status switch A1 (clock/ program)
*/


//clock en modeltijd
#define tday 15 //tday how long is a modeltimeday in minute 24 is good value lager dan 10 werkt het geheel niet goed
byte PRG_min[32]; //Time next active minute
byte PRG_hr[32]; //Time next actice hour
unsigned long Clk;
unsigned int mt; //modeltime minute
byte mt_min=1; //modeltimeclock minutes 
byte mt_hr=0; //modeltimeclock hours
byte mt_zonop=7; 
byte mt_zononder = 21; 
//Zorg dat bij programmeren altijd een BASIS adres wordt genomen. dus 1=(1-4)2=(5-8) 3=(9-12) 4=(13-16) 5 6 enz 

//basic adres, adres Daylight decoder DL +1 
//VL (verlichting) decoder 16 adresses higher
//EV (events) decoder 16adresses higher

int DL_adresmin;
int VL_adresmin;
int EV_adresmin;


volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
boolean DEK_Monitor = false; //shows DCC monitor in serial monitor. NOTE: true will take up to 400bytes of memory, program can run out of memory and crash.
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

	//test mode
	delay(350);
	Serial.begin(9600);


	DDRB |= (1 << 5);	//pin13
	DDRB |= (1 << 4);  //Pin12 als output
	DDRB |= (1 << 3); //PIN11 as output

	DDRC &= ~(1 << 0); //set PINA0 as input
	DDRC &= ~(1 << 1); //set PINA1 as input


	Clk = millis();

	//Fastled part
	//eerste regel mee gespeeld veel opties die ik nog niet begriijp
	//FastLED.addLeds<ledtype, LPdl,GRB> (led_dl, led_OW*led_NZ).setCorrection(TypicalLEDStrip); //create strip of

	FastLED.addLeds<NEOPIXEL, LPdl>(led_dl, 240);//create strip of 32leds on pin7 'verlichting' Xx leds on pin 8 'Daglicht'
	FastLED.addLeds<NEOPIXEL, LPvl>(led_vl,32);//create strip of 32leds on pin7 'verlichting'
	//FastLED.addLeds<NEOPIXEL, LPev>(led_ev, 16);//create strip of 16 leds on pin6 'Events'

	//FastLED.setMaxPowerInVoltsAndMilliamps(5, 7000);
	
	//COM_reg |= (1 << 4);//register bit schakelt alle effecten uit bij sunrise en sunset, true is uit, false = aan schakelen met een CV 
	
	//instellen modeltijd bij power up
	mt_hr = mt_zonop;
	mt_min = 1;

	randomSeed(analogRead(0));


	VL_adresmin = (COM_DCCAdres * 4) + 1; //no mistake, COM_DCCadres for decoder = 1 lower, COM_DCCadres+1 1th led adres.
	//VL_adresmin = DL_adresmin + 64;
	//EV_adresmin = VL_adresmin + 64;

	//FastLED.setMaxPowerInVoltsAndMilliamps(5, 8000);	
	//FastLED.setBrightness (255); //beter niet

	//DeKoder part, interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1

	//program part
	//Set register 
	COM_reg &= ~(1 << 1); //clear bit1, separate ledstrips of layout width.
	//COM_reg |= (1 << 1); //set bit 1 ledstrip as 1 pcs folded over the layout.

	mt = (tday * 1000) / 24;
	setup_leds();
	if (DEK_Monitor==true) Opening();
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
	//Daglicht...
	//gebruikte leds instellen, straks ook met CV in te stellen
	//Programs VL:          program number		program outputs			assigned leds
	//PRG_traffic			12					1,2						vl0; vl1
	led_vlap[0] = 1;
	led_vlap[1] = 2;

	//programs DL
	//PRG_Flashlight			11						ev2; ev6

	//check EEprom for modyfied entry
	for (int i = 0; i < 64; i++) {
		//if (EEPROM.read(i) < 0xFF) led_dlap[i] = EEPROM.read(i);
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
	Serial.println(DL_adresmin+63);
	Serial.print("adressen verlichting leds van ");
	Serial.print(VL_adresmin);
	Serial.print(" tot en met ");
	Serial.println(VL_adresmin+63);
	Serial.print("adressen gebeurtenissen leds van ");
	Serial.print(EV_adresmin);
	Serial.print(" tot en met ");
	Serial.println(EV_adresmin+63);
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
void COM_resetEEprom(int start) {
	//resets EEprom to 0xFF from start to start+64
	//Reloads led assign to predifined values
	for (int i = start; i < start+64; i++) {
		if (EEPROM.read(i) < 0xFF)EEPROM.write(i, 0xFF);
		setup_leds();
	}
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
	if (DEK_Monitor==true) APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
	APP_COM(type, adres, decoder, channel, port, onoff, cv, value);
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
		
		PINB |=(1 << 3); // toggle led on pin 11, toont of de clock loopt....
		
		if (mt_min > 60) {
			mt_min = 1;
			mt_hr ++;
				Serial.print("Modeltijd uur:  ");
				Serial.println(mt_hr);

			if (mt_hr > 23) {
				mt_hr = 0;

			}
		}
	}
}
void COM_ProgramAssign() {
	//plays, assigns programs only 1 every cycle 
	//first 64 active programs 1by1 then only 1 not active programs, then 64 active and so on. 	
	//after start all programs must be passed once
	static byte init = 0;
	if (init == 0) {
		for (byte i = 0; i < 32; i ++ ) {
			PRG_reg[i] = 0; //init program registers
			switch (i) {
			case 2:
				PRG_hr[2] = mt_zonop;
				PRG_min[2] = 1;
				PRG_reg[2] |= (1 << 2);
				break;
			case 3:
				//no initial start

				break;
			case 4:
				//no initial start
				
				break;
			default:
				COM_ps(i);
				break;
			}			
		}
		init = 1;
	}
	
	static byte pa; //program active
	static byte pna;//program not active

	if (bitRead(COM_reg, 0) == true) { //find active program

		if (bitRead(PRG_reg[pa], 0) == true) COM_ps(pa);
		pa ++;
		if (pa > 64) {
			pa = 0;
			COM_reg &= ~(1 << 0); //reset bit 0, next cycle not active
		}
	}
	else { //find not-active program
		//prg_reg bit 3 = time switched program

		if (bitRead(PRG_reg[pna], 0) == false & bitRead(PRG_reg[pna], 2) == true & PRG_hr[pna] == mt_hr & PRG_min[pna] == mt_min) {
			//schakelen op modeltijd
			switch (pna) {
			case 2:
				if (mt_hr == mt_zonop) {
					Serial.println("zon op");
					COM_reg &= ~(1 << 3);
				}
				else {
					COM_reg |= (1 << 3);				
					Serial.println("zon onder");
				}
				PRG_reg[2] |= (1 << 0);
				PRG_reg[2]|= (1 << 7);
				break;
			}
			COM_ps(pna);
		}

		COM_reg |= (1 << 0); //next cycle active
		pna ++;
		if (pna > 64)pna = 0;
	}
}
void COM_ps(int pn) { //ps=program switch
	//total, max 64 programs
	//1-10 daylight and weather 11/30 lighting houses and streetlights  31 > no idea yet
	//Serial.println(bitRead(PRG_reg[3], 0));
	switch (pn) {
	case 2: //schakel daglicht in of uit
		PRG_dl(pn);
		break;
	case 3:
		PRG_dld(pn);
	case 4:
		PRG_lightning();
		break;
	case 11:
		//PRG_flashlight(pn);
		break;
	case 12:
		PRG_traffic(pn);
		break;

	default:
		//all not defined programs// do nothing
		break;
	}
}
void COM_switch() {
	//handles the manual switches in the project
	static long Sw_time = 0;
	static unsigned int count = 0;
	static boolean dld = 0;

	if (millis() - Sw_time >50) { //every xxms
		Sw_time = millis(); //reset counter
		//test switch A0 dag/nacht

		if (bitRead(PINC, 0) == false) { //switch A0 pressed			
			if (bitRead(SW_reg, 0) == false) {
					SW_reg |= (1 << 0); //set switchstate
			}
			else { //switchstate true so button was allready pressed, wait for 2 seconds
				count++;	

				if (count > 40 & dld==false ) { //button hold > 2 seconds
					//Serial.println("lang ingedrukt");
					dld = true;
					COM_reg ^= (1 << 3); //bit3 false = day, true is night					
					PRG_reg[3] |= (1 << 0);//start program 3	
					//PRG_reg[3] |= (1 << 1); 
					PRG_reg[2] &= ~(1 << 0); //Stop program 2, 
					PRG_reg[3] |= (1 << 7); //flag program 3 changed

					PRG_reg[2] &= ~(1 << 2); //disable modeltime start
					
					
					if (bitRead(COM_reg, 3) == false) {
						PORTB |= (1 << 5);
						PORTB &= ~(1 << 4);
					}
					else {
						PORTB |= (1 << 4);
						PORTB &= ~(1 << 5);
					}

				}
			}
		}
		else { //button released			

			if (bitRead(SW_reg, 0) == true) {

				if (dld==false) {

					PRG_reg[2] |= (1 << 0); //starts program 2 PRG_dl()
					PRG_reg[2] |= (1 << 7); //flag for changed
					COM_reg ^= (1 << 3); //bit3 false = day, true is night
					PORTB &= ~(1 << 4);
					PORTB &= ~(1 << 5);
				}
				count = 0;
				dld = false;
				SW_reg &= ~(1 << 0); //reset switch state
			}
		}
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
void APP_COM(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//Basis instellingen van de decoder

	if (decoder==COM_DCCAdres) {
		if (type == false) { //switch

		}
		else { //CV
//Serial.println(decoder);
			switch (cv) {
			case 8: //request reset eeprom
				switch (value) {
				case 10: //reset EEprom 0-63
					COM_resetEEprom(0);
					break;
				case 20://reset EEprom 64-127
					COM_resetEEprom(64);
					break;
				case 30: //reset EEprom 128-191
					COM_resetEEprom(128);
					break;
				}
				break;
			}
		}
	}
}
void APP_DL(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
//Daglicht 64leds
	if (adres >= DL_adresmin & adres <= DL_adresmin+63) {
		if (port == true) {
			PORTB |= (1 << 4);
			led_dl[adres-DL_adresmin]= 0xCC2222; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
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

	//******AANTALLEN en adressen aanpassen straks....
	int dec;
	if (adres >= VL_adresmin & adres <= VL_adresmin+63) {
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
		else {//CV
		dec = 4*(decoder - (COM_DCCAdres)-17); //geeft led volgorde van led op channel 00 van deze (sub)decoder		
		//Serial.println(dec);
		switch (cv) { //adres is altijd hier het adres van de decoder. Niet van het channel
		case 10:
			dec = dec + 0;
			led_vlap[dec] = value;
			EEPROM.write(dec + 64, value);
			break;
		case 11:
			dec = dec + 1;
			led_vlap[dec] = value;
			EEPROM.write(dec + 64, value);
			break;
		case 12:
			dec = dec + 2;
			led_vlap[dec] = value;
			EEPROM.write(dec + 64, value);
			break;
		case 13:
			dec = dec + 3;
			led_vlap[dec] = value;
			EEPROM.write(dec + 64, value);
			break;
		}

		}
	}
}

void APP_EV(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//Events 64leds
	//******AANTALLEN en adressen aanpassen straks....

	if (adres >= EV_adresmin & adres <= EV_adresmin+63) {
		if (port == true) {
			PORTB |= (1 << 5);
			led_ev[adres - EV_adresmin] = 0xFFFFFF; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
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

	switch (group) {
	case 0:
		led_dl[output] = CRGB(r, g, b);
		break;
	case 1:
		for (byte i = 0; i < 65; i++) { //check all leds in this group
			if (led_vlap[i] == output)led_vl[i] = CRGB(r, g, b);			
		}
FastLED.show();
		break;
	case 2:
		break;
	}
}


void PRG_dl(byte pn) {	
	//prg_reg bit0=init bit1=active bit2=timerstart bit7=changed bit4=sunrise,sunset in motion
	//bit 4-7 exclusive for this program
	//bit5
	//bit 6 flag voor bereiken eindwaarde
	//bit7=flag for manual switched true is switched

	//schakeld daglicht=program 2	

	static byte weer=3; //welk weertype voor komende dag...
	static byte fxb; //hoe ver het effect naar het westen
	static byte rled[3]; //willekeurige byte
	static byte sp = 1; 
	
	/*
	1=zon
	2=half bewolkt, af en toe zon
	3=bewolkt
	4=donker, regenachtig
	*/
	static byte maxclr[3]; //max te bereiken kleuren per weer type
	static byte ledcount = 0; // welke led
	static byte sc = 0; //laatste led, en teller in fase 113
	static byte teller = 0; //algemene teller
	static byte s = 0; //shift, aantal over geslagen leds dynamisch
	static byte st = 0; //shift overgeslagen leds statisch

	static long t = 0;
	static byte periode = 1;
	static byte fase = 0; 
	static byte nextfase = 0;

	static byte al = 0; //al=aantal leds
	
	if (bitRead(PRG_reg[pn], 7) == true){ 
		//if true day/night is switched
		PRG_reg[pn] &= ~(1 << 7);
		teller = 0;
		ledcount = 0;
		fase = 0;
		periode = 1;

		if (bitRead(COM_reg, 3) == false) {
			PORTB |= (1 << 5);
			PORTB &= ~(1 << 4);
			mt_hr = mt_zonop; //reset model tijd
		}
		else {
			PORTB |= (1 << 4);
			PORTB &= ~(1 << 5);
			mt_hr = mt_zononder;
		}
	}
	if (millis() - t > periode) {
		t = millis();	
//*****************************switch fase
			switch (fase){
			case 0: 
				al = led_NZ*led_OW - 1; //al=aantal leds

				if (bitRead(COM_reg, 3) == false) { //sunrise

					if (bitRead(COM_reg, 4) == false) { //general with or without sunset en sunrise effects
						//weer = random(1, 4); //bepaal weertype
						weer = 2; //during debug

					}
					else {
						weer = 3;
					}
					Serial.print("Weertype: ");
					Serial.println(weer);


					switch(weer) {
					case 1: //zonnig
						fase = 10;
						periode = 1;
						st = random(1, 5+1); //aantal leds wat wordt overgeslagen. 
						fxb = random(led_NZ, al * 7/10);
						break;

					case 2: //half bewolkt, vaak zon.
						fxb = random(led_NZ, al * 7 / 10);
						fase = 20;
						break;

					case 3: //bewolkt weer geen kleureffecten
						fase = 40;
						if (bitRead(COM_reg, 4) == false) {
							maxclr[2] = random(170, 210);
							maxclr[0] = maxclr[2] - random(10, 50); maxclr[1] = maxclr[2] - random(0, 30);
							//Serial.print("rood= ");
							//Serial.print(maxclr[0]);
						}
						else { //no effect
							maxclr[0] = 240; maxclr[1] = 240; maxclr[2] = 240;
						}						
						periode=1;
						teller = 0;
						break;
					}
				}
				else { //sunset

					switch (weer) {
					case 1:
						fase = 100;
						st = random(0, 5); //aantal leds wat wordt overgeslagen. 
						fxb = random(al * 4 / 10, al - led_NZ);
						maxclr[0] = random(60, 120);
						maxclr[1] = maxclr[0];
						maxclr[2] = maxclr[0];
						teller = random(1, 50);
						break;

					case 2: //regenachtige bewolkt, kans op onweer
						fase = 120;
						fxb = random(al * 4 / 10, al - led_NZ);
						teller = 0;
						//hier random onweer inschakelen
						//voorlopig even altijd, dus...
						PRG_reg[4] |= (1 << 0); //start prg_lightning
						break;

					case 3:
						fase = 140;
						periode = 1;
						teller = 0;
						break;
					}					
				}
				break;

			case 1:
				//stops program
				PRG_reg[pn] |= (1 << 7); 
				break;

//************WEER 1 ZONNIG

			case 10: //susnrise much effect				
				if (rled[0] == ledcount) {
					if (ledcount - sc > st ) {
						sc = ledcount;
						if (ledcount < fxb) led_dl[ledcount]= CRGB (3, 0, 0);
					}
				}
				if (ledcount==rled[1]) led_dl[ledcount]= CRGB(20, 20, 20);
				teller++;
				if (teller>50) {
					teller = 0;
					fase = 11;	
					maxclr[0] = random(20, 200); //te bereiken rode kleur instellen
					maxclr[1] = maxclr[0] * 3/ 10;
				}
				break;

			case 11:
				if (ledcount - sc > st) {
					sc = ledcount;
					if (ledcount < fxb) led_dl[ledcount].r += 2;
				}
				if (led_dl[ledcount].r >= maxclr[0])fase = 12;
				if (ledcount == rled[0]) wit(rled[0], 2, maxclr[0], maxclr[1], maxclr[2],false);
				break;

			case 12:
				//rood faden naar geel, witte licht op laten komen
				//per cycle 1 led willekeurig witter maken
				if (ledcount - sc > st) {
					sc = ledcount;
					if (ledcount < fxb & led_dl[ledcount].g < maxclr[1]) {
						led_dl[ledcount].g += 1;
						PRG_reg[pn] &= ~(1 << 6);
					}					
				}
				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true)) {
					fase = 13;
					maxclr[0] = 240; maxclr[1] = 240; maxclr[2] = 240;
				}
				if (ledcount == rled[0]) wit(rled[0], 6, maxclr[0], maxclr[1], maxclr[2],false);
				if (ledcount == rled[1])led_dl[ledcount] = CRGB(180, 90, 0);
				break;

			case 13:
				//white out
				wit(ledcount, sp, maxclr[0], maxclr[1], maxclr[2],true);
				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true)) fase = 1;
				if (led_dl[ledcount].b > 50) sp = 2;
				break;

//******************WEER 2
			case 20: //bewolkt zonnig weer. gevlekt eindresultaat?
				//als 40 maar met gekleurde leds af en toe
				fase = 21;
				maxclr[0] = 245; maxclr[1] = 245; maxclr[2] = 255;
				break;

				case 21:
					if (ledcount == rled[0] | ledcount == rled[1]) {
						wit(ledcount, 6, maxclr[0], maxclr[1], maxclr[2], false);
						FastLED.show();
					}

					//if (ledcount == rled[1] & ledcount < fxb & bitRead(PRG_reg[pn], 5) == true) led_dl[ledcount] = CRGB(250, 0, 0);
					if (ledcount == rled[2] & bitRead(PRG_reg[pn], 4) == true) led_dl[ledcount] = CRGB(250, 130, 0);

					if (ledcount >= al) teller++;

					
					if (teller > (al*7 / 10)) {
						teller = 0;
						fase = 41;
						maxclr[0] = random(180, 250);
						maxclr[1] = maxclr[0]; maxclr[2] = maxclr[0];
					}
					else {
						if (teller > (al*1 / 10) & teller < (al*2/10)) {
							if (ledcount == rled[1] & ledcount < fxb) led_dl[ledcount] = CRGB(250, 0, 0);
						}
						if (teller > (al * 2 / 10) & teller < (al*4 / 10)) {
							if (ledcount == rled[2]) led_dl[ledcount] = CRGB(250, 130, 0);
						}						
					}
					break;

//***************WEER 3 niet ingevuld

//**********WEER 4
			case 40: //weertype 4, slecht weer naar donker daglicht
				
				if (ledcount == rled[0] | ledcount == rled[1] | ledcount==rled[2]) {
					wit(ledcount, 6, maxclr[0], maxclr[1], maxclr[2], false);
					FastLED.show();
				}

				if (ledcount >= al) teller++;

				if (teller > (al/2)) { //var calc in fase 4
					fase = 41;
					periode = 1;
					teller = 0;
				}
				break;

			case 41: //to full daylight
				wit(ledcount, 1, maxclr[0], maxclr[1], maxclr[2], true);
				if (ledcount >= al & bitRead(PRG_reg[pn], 6) == true) fase = 1;
				break;

//******************SUNSET******SUNSET********SUNSET*******
			case 100: //sunset
				//fase = 210;
				//nextfase = 110;
				fase = 110;
				break;

			case 110:
				if (ledcount>=al) teller++;
				if (teller < 150) {
					zwart(ledcount, 1, 20, 20, 20, false,false);
				}
				else {
					fase = 111;
					maxclr[1] = maxclr[0] * 3/ 10;
					teller = 0;
				}
				break;

			case 111:
				zwart(ledcount, 1, 5, 5, 5, false,false);
				if (ledcount > fxb) {
					if (ledcount - sc > st) {
						sc = ledcount;
						zwart(ledcount, 2, 200, maxclr[1], 0, true,false);
					}
					else {
						zwart(ledcount, 1, 10, 10, 10, false,false);
						if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);						
					}
				}
				else {
					zwart(ledcount, 1, 10, 10, 10, false,false);
					if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
				}
				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true))fase = 112;
				break;

			case 112:
				if (ledcount >= al) {
					teller ++;					
				}

				if (teller < 50) {					
					zwart(ledcount, 1, 3, 3, 3, false,false);
					if (ledcount == rled[0])led_dl[ledcount] = CRGB(50, 20, 0);
					if (ledcount == rled[1]) led_dl[ledcount] = CRGB(0, 0, 0);							
				}
				else {			
					fase = 113;
				}		
				break;

			case 113:			
				if (ledcount > fxb) {
					if (ledcount - sc > st) {
						sc = ledcount;
						zwart(ledcount, 1, 200, 0, 0, true,false);
					}
				}

				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true))fase = 114; //stop
				if (ledcount == rled[0]) led_dl[ledcount] = CRGB(0, 0, 0);

				break;

			case 114:
				zwart(ledcount,1, 2, 2, 2, true,false);
				if (ledcount == rled[0]) led_dl[ledcount] = CRGB(0, 0, 0);
				if (ledcount == rled[1]) led_dl[ledcount] = CRGB(0, 0, 0);
				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true))fase = 200; 
				break;

			case 120:	 //sunset half bewolkt			
				maxclr[0] = random(40, 90); maxclr[1] = maxclr[0]; maxclr[2] = maxclr[0];
				//fase = 210;
				//nextfase = 122;
				fase = 122;
				break;

			case 122:
				zwart(ledcount, 2, maxclr[0], maxclr[1], maxclr[2], true, false);
				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true))fase = 123;
				if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
				break;

			case 123:
				if (ledcount >=al) teller++;
				if (teller < (al*2/10)) {
					zwart(ledcount, 1, 5, 5, 5, false, false);
					if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
					if (ledcount == rled[2] & ledcount > fxb)led_dl[ledcount] = CRGB(90, 30, 0);
					if (ledcount == rled[1])led_dl[ledcount] = CRGB(100, 30, 0);
				}
				else {
					fase = 200;
				}
				break;

			case 140: //begin sunset bewolkt weer, geen effecten
				//fase = 210;
				//nextfase = 141;
				fase = 141;
				break;

			case 141:
				zwart(ledcount, 1, 3, 3, 3, true,false);
				if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true)) {
					fase = 200;
					periode = 30;
				}
				break;

			case 200:
				if (ledcount == 0) teller = 0;
				if (ledcount == rled[0]) led_dl[ledcount] = CRGB(0, 0, 0);
				if (ledcount == rled[1]) led_dl[ledcount] = CRGB(0, 0, 0);
				if (ledcount - (teller * led_NZ) > led_NZ)teller++;
				

				if (ledcount >= (led_NZ*teller) & (ledcount <= led_NZ*teller + led_NZ)) {
					zwart(ledcount, 1, 1, 1, 1, true, true);
				}

				if ((ledcount >= al) & (bitRead(PRG_reg[pn], 6) == true)) {
					fase = 201; 
					periode = 50;
					teller = 0;
					st = random(2, 4);
				}
				break;

			case 201:
				//naar nacht
				if (ledcount - sc > st) {
					sc = ledcount;
					if (led_dl[ledcount].r +led_dl[ledcount].g + led_dl[ledcount].b ==0) {
						led_dl[ledcount +1] = CRGB(0, 0, 0);
					}
					else {
						led_dl[ledcount] = CRGB(0, 0, 0);
					}
				}
				led_dl[rled[0]] = CRGB(0, 0, 0);
				if (ledcount >= al) {
					teller++;
					if (teller > 1) fase = 1;
				}
				PORTB ^= (1 << 4);
				FastLED.show();
				break;

			case 210: //to early stop not needed sunset
				wit(ledcount, 0, 2, 2, 2, true);

				if (ledcount >= al) {
					Serial.println(bitRead(PRG_reg[pn], 6));

					if (bitRead(PRG_reg[pn], 6) == true) {
						fase = nextfase;
					}
					else {
						fase = 1;
					}
				}
				break;
}
//**********************end switch fase
		
			ledcount++;
			if (ledcount > al){//merk op 1 verder als in de switch case
				FastLED.show();
				ledcount = 0;
				PRG_reg[pn]|= (1 << 6); //flag voor bereiken eindwaarde				
				rled[0] = random(0, al); 
				rled[1] = random(0, al);
				rled[2] = random(0, al); 
				randomSeed(analogRead(A5));	//port (pinA5) must be free, not in use
				
				sc = 0;
				s++;
				if (s > 5) s = 0;

				if (bitRead(COM_reg, 3) == false) { //flashes the led status
					PORTB ^= (1 << 5);
				}
				else {
					PORTB ^= (1 << 4);
				}
			}	

			//STOP PROGRAM (bit7 of register = true)
			if (bitRead(PRG_reg[pn],7)==true) { //stop this program plus init all variables
				Serial.println("Stop program");
				PRG_reg[pn] &= ~(1 << 7);
				ledcount = 0;
				sc = 0;
				s = 0;
				fase = 0;	
				PRG_reg[pn] &= ~(1 << 0);

				PRG_reg[pn] |= (1 << 2); //enable model time start
				if (bitRead(COM_reg, 3) == false) { //stops flashing led
					PORTB |= (1 << 5);
					PRG_hr[2] = mt_zononder;
				}
				else {
					PORTB |= (1 << 4);
					PRG_hr[2] = mt_zonop;
				}
			}
	}
}
void wit(byte led, byte inc, byte mr, byte mg, byte mb,boolean stop) {

	if (led_dl[led].r < mr) {
		led_dl[led].r = led_dl[led].r + inc;
		if (stop==true) PRG_reg[2] &= ~(1 << 6);
	}
	if (led_dl[led].g < mg) {
		led_dl[led].g = led_dl[led].g + inc;
		if(stop==true) PRG_reg[2] &= ~(1 << 6);
	}
	if (led_dl[led].b < mb) {
		led_dl[led].b = led_dl[led].b + inc;
		if(stop==true)PRG_reg[2] &= ~(1 << 6);
	}
}
void zwart(byte led, byte dec, byte mr, byte mg, byte mb, boolean stop,boolean nacht) {
	static byte temp;
	temp = dec;
	if (led_dl[led].r > mr) {
		if (led_dl[led].r < dec)temp = led_dl[led].r;
		led_dl[led].r = led_dl[led].r - temp;
		temp = dec;
		if (stop == true) PRG_reg[2] &= ~(1 << 6);
	}
	if (led_dl[led].g > mg) {
		if (led_dl[led].g < dec)temp = led_dl[led].g;
		led_dl[led].g = led_dl[led].g - temp;
		if (stop == true) PRG_reg[2] &= ~(1 << 6);
	}
	if (led_dl[led].b > mb) {
		if (led_dl[led].b < dec)temp = led_dl[led].b;
		led_dl[led].b = led_dl[led].b - temp;
		if (stop == true)PRG_reg[2] &= ~(1 << 6);
	}

		
	//correctie te weinig van 1 kleur, zwart blijft zwart
	if ((led_dl[led].r + led_dl[led].g + led_dl[led].b) > 0 & nacht==true) {
		if (led_dl[led].r < mr) {
			led_dl[led].r++;
			if (stop == true)PRG_reg[2] &= ~(1 << 6);
		}

		if (led_dl[led].g < mg) {
			led_dl[led].g++;
			if (stop == true)PRG_reg[2] &= ~(1 << 6);
		}
		if (led_dl[led].b < mb) {
			led_dl[led].b++;
			if (stop == true)PRG_reg[2] &= ~(1 << 6);
		}
	}

	
}
void PRG_lightning() { //Programnummer=4
	if (bitRead(PRG_reg[4], 0) == true) {
		PRG_reg[4] &= ~(1 << 0);
		Serial.println("lightning ingeschakeld");
	}


}
void PRG_dld(byte pn) {
	//switches between day and night without sunrise or sunset		
	static unsigned long t = 0;
	static byte lc=0;

	if (millis() - t > 5) {
		t = millis();

		if (bitRead(COM_reg, 3) == false) { //dag
			led_dl[lc] = CRGB(250, 250, 250);
			mt_hr = mt_zonop;
			mt_min = 0;
		}
		else {//nacht	
			led_dl[lc] = CRGB(0, 0, 0);
			mt_hr = mt_zononder;
			mt_min = 0;		

		}
		lc++;
		FastLED.show();
		if (lc > (led_NZ*led_OW) - 1) {
			lc = 0;
			PRG_reg[pn] &= ~(1 << 0); 

			if (bitRead(COM_reg, 3) == true) PRG_reg[4] |= (1 << 0);//lightning starten? alleen tijdens onwikkeling

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
			//FastLED.show();
		}
	}
}
void loop() {
	COM_Clk();
	COM_ProgramAssign();
	DEK_DCCh();
	COM_switch();
}
