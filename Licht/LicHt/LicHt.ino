/*
 Name:		LicHt.ino
 Created:	2018
 Author:	Rob Antonisse
 Version: 1.01

 LicHT is a project for full automated lighting of the model railroad, including day and night cyclus.
 Based on WS2811 adressable led control.

 Many thanks to creators of the FASTLED library more info on: http://fastled.io/
 Many thanks for creators of the WS2811 and WS2812B chips and protocol WorldSemi more info: http://www.world-semi.com/

*/

#include <FastLED.h>
#include <eeprom.h>

byte COM_DCCAdres;
byte COM_DCCmode;
byte led_al = 0xFF;
byte Pmax; //to reduce calculations Pmax=Led_al-1
byte tday = 0xFF; //#501 CV4, tday how long is a modeltimeday in minute 24 is good value lager dan 10 werkt het geheel niet goed
byte DL_wt = 0xFF;//#502 CV6, Weertype

byte mt_zonop = 0xFF; //EEPROM #503
byte mt_zononder = 0xFF; //EEPROM #504 

byte Srspeed = 0xFF; //factor 0-10 
unsigned int SrS; //#510 CV5, sun rise speed in micros/40 

//tbv display
byte shft[2];
byte klok[4]; //0=minute low 1=minute dec 2= hr 3=hr dec

CRGB led_dl[240]; //max adressable pixels in daylight not editable
CRGB led_vl[40]; //max adressable pixels in verlichting not editable
//tbv van prg_lightning
CRGB led_lgt[12]; //max aantal leds voor bliksem, dit dient  als geheugen voor de 'oude'waarde van de led
CRGB led_fx[8]; //array for effects pixels

byte led_vlap[48]; //vlap=verlichting assign program, assigns output of a  program to a pixel (40) 32 in led_vl 8 in led_fx

byte COM_reg;

byte COM_set = 0xFF;//bit0=Lighting on(true) or off(false)
byte PRG_reg[32]; //prg_reg 
byte SW_reg;
byte SW_old;
byte SW_new;
byte SW_change;
unsigned long Sw_time;
byte SW_count;
byte prgedit;
byte PRG_min[32]; //Time next active minute
byte PRG_hr[32]; //Time next actice hour
byte blink[2]; //tbv blinking leds in Traffic

unsigned long Clk;
unsigned long FastClk;
unsigned int mt; //modeltime minute

byte mt_min; //modeltimeclock minutes 
byte mt_hr; //modeltimeclock hours

volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts

byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[6]; //registerbyte for 12 command buffers (terug naar 6 buffers), bit7= free (false) bit0= CV(true)
byte DEK_Buf0[6];
byte DEK_Buf1[6];
byte DEK_Buf2[6];
byte DEK_Buf3[6];
byte DEK_Buf4[6];
byte DEK_Buf5[6];

void setup() {
	//Serial.println("setup");
	Serial.begin(9600);
	DDRB |= (1 << 5);	//pin13
	DDRB |= (1 << 4);  //Pin12 als output
	//tbv clock display
	DDRD |= (1 << 4); //serial data pin as output (PIN4)
	DDRD |= (1 << 5); //shift clock output(PIN5)
	DDRD |= (1 << 6);//Shift latch output(PIN6)
	//GPIOR0 = 0; //general purpose register used for flags and booleans
	PORTD = 0; //reset portD	
	PORTD |= (1 << 5);
	PORTD |= (1 << 6);
	GPIOR0 |= (1 << 1); //enable void DSP_shift een malig	
	//DeKoder part, interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1	
	//factory reset
	SW_old = PINC;
	if (SW_old == B00000110) {
		PORTB |= (1 << 5);
		PORTB |= (1 << 4);
		MEM_reset(0, EEPROM.length());
		delay(1000);
		PORTB = 0;
	}
	//initialise
	Clk = millis();
	randomSeed(analogRead(A5));
	//COM_DCCAdres = 64; //basic DCC adress
	byte temp;
	if (EEPROM.read(100) > 200)EEPROM.write(100, 64); //basic decoder adres, max 200
	COM_DCCAdres = EEPROM.read(100);
	if (EEPROM.read(101) > 2)EEPROM.write(101, 0);
	COM_DCCmode = EEPROM.read(101); //DCC mode
	COM_set = EEPROM.read(400);
	for (byte i = 0; i < 8; i++) {
		switch (i) {
		case 1: //lasser 1
			if (bitRead(COM_set, 1) == true) {
				PRG_reg[2] |= (1 << 1);
			}
			else {
				PRG_reg[2] &= (1 << 1);
			}
			break;
		case 2: //lasser 2
			if (bitRead(COM_set, 2) == true) {
				PRG_reg[3] |= (1 << 1);
			}
			else {
				PRG_reg[3] &= (1 << 1);
			}
			break;
		case 3: //fire
			if (bitRead(COM_set, 3) == true) {
				PRG_reg[4] |= (1 << 1);
			}
			else {
				PRG_reg[4] &= (1 << 1);
			}
			break;
		case 4: //tv
			if (bitRead(COM_set, 4) == true) {
				PRG_reg[6] |= (1 << 1);
			}
			else {
				PRG_reg[6] &= (1 << 1);
			}
			break;
		}
	}
	temp = EEPROM.read(500);
	if (temp == 30 | temp == 60 | temp == 90 | temp == 120 | temp == 150 | temp == 180 | temp == 210 | temp == 240) {
		led_al = temp;
	}
	else {
		//default value
		EEPROM.write(500, 240);
		led_al = 240;
	}
	//Pmax = led_al - 1;
	FastLED.addLeds<NEOPIXEL, 8>(led_dl, 240);//leds on pin 8 'Daglicht' declaration max possible pixels
	FastLED.addLeds<NEOPIXEL, 7>(led_vl, 40);//create strip of 32leds on pin7 'verlichting'
	FastLED.addLeds<NEOPIXEL, 9>(led_fx, 8);//create strip of 8 pixels for effects on PIN9	
	if (EEPROM.read(501) < 10 | EEPROM.read(501) > 60) EEPROM.write(501, 24);
	tday = EEPROM.read(501); //duur van model-tijd dag instellen
	COM_tday();
	if (EEPROM.read(502) > 4) EEPROM.write(502, 0); //Weertype 0=random 1=zon 2=halfbewolkt 3=bewolkt 4=no effects
	DL_wt = EEPROM.read(502);
	if (EEPROM.read(503) < 5 | EEPROM.read(503) > 10) EEPROM.write(503, 7);
	mt_zonop = EEPROM.read(503); //Model-tijd zonsopgang tijd
	if (EEPROM.read(504) < 18 | EEPROM.read(504) > 22) EEPROM.write(504, 20);
	mt_zononder = EEPROM.read(504); //Model-tijd Zonsondergang tijd
	//snelheid fx zonsopgang in 10 stappen, recht evenredig aan aantal leds
	if (EEPROM.read(510) < 1 | EEPROM.read(510) > 10) EEPROM.write(510, 5); //5=medium speed
	Srspeed = EEPROM.read(510);
	COM_SrS();
	//nieuwe vlapper 11sept2018
	for (int i = 0; i < 48; i++) {
		if (EEPROM.read(i) > 47){// == 0xFF) {
			EEPROM.write(i, i);
		}
		led_vlap[i] = EEPROM.read(i);
	}
	GPIOR0 |= (1 << 2); //enable effects
	PRG_reg[0] |= (1 << 0); //starts program 2 PRG_dl()
	PRG_reg[0] |= (1 << 7); //flag for changed
	COM_rt(); //resets the building model time depended programs
}
void MEM_reset(int start, int aantal) {
	for (int i = start; i < start + aantal; i++) {
		if (EEPROM.read(i) < 0xFF)EEPROM.write(i, 0xFF);
	}
	delay(500);
}
void MEM_change() { //called when mem is changed, check all memories

	if (EEPROM.read(101) != COM_DCCmode) EEPROM.write(101, COM_DCCmode);
	if (EEPROM.read(400) != COM_set) EEPROM.write(400, COM_set);

	if (EEPROM.read(500) != led_al) { //qty pixels changed
		EEPROM.write(500, led_al);
		COM_SrS();
	}
	if (EEPROM.read(501) != tday) { //duration modeltime 
		EEPROM.write(501, tday);
		COM_tday();
	}
	if (EEPROM.read(502) != DL_wt)EEPROM.write(502, DL_wt);
	if (EEPROM.read(503) != mt_zonop) EEPROM.write(503, mt_zonop);
	if (EEPROM.read(504) != mt_zononder) EEPROM.write(504, mt_zononder);

	if (EEPROM.read(510) != Srspeed) { //sunrise speed changed
		EEPROM.write(510, Srspeed);
		COM_SrS();
	}
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
	DEK_countPA = 0;
	DEK_Reg = 0;
	DEK_Status = 0;
	for (int i = 0; i < 6; i++) {
		DEK_byteRX[i] = 0; //reset receive array
	}
}
void DEK_BufCom(boolean CV) { //create command in Buffer
	byte i = 0;
	while (i < 6) {

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
void DEK_BitRX() {
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0;
	//static byte n = 0; //7juni weggehaald, functie niet duidelijk
	DEK_Reg |= (1 << 4);//resets and starts process if not reset in this void
	switch (DEK_Status) {
		//*****************************
	case 0: //Waiting for preample 
		if (bitRead(DEK_Reg, 3) == true) {
			DEK_countPA++;
			if (DEK_countPA > 12) {
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
		COM_dek(bitRead(DEK_BufReg[n], 0), decoder, channel, port, onoff, cv, value);
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
	if (n > 6)n = 0; //was > 12
}
void COM_dek(boolean type, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//type=CV(true) or switch(false)
	//decoder basic adres of decoder 
	//channel assigned one of the 4 channels of the decoder (1-4)
	//Port which port R or L
	//onoff bit3 port on or port off
	//cv cvnumber
	//cv value
	int adres;
	adres = ((decoder - 1) * 4) + channel;
	//APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
	APP_COM(type, adres, decoder, channel, port, onoff, cv, value);
	APP_VL(type, adres, decoder, channel, port, onoff, cv, value);
}
void COM_Clk() {
	//static byte klokteller = 0;
	if (millis() - Clk > mt & bitRead(GPIOR0, 6) == false) {
		Clk = millis();
		mt_min++;
		if (mt_min > 59) {
			mt_min = 0;
			mt_hr++;
			if (mt_hr > 23) {
				mt_hr = 0;
			}
			//Serial.print(F("MT uur:  "));
			//Serial.println(mt_hr);			
		}
		DSP_clock();
	}
}
void COM_ProgramAssign() {
	if (bitRead(GPIOR0, 5) == false) {//enable normal mode
		static byte pa; //program active
		static byte pna;//program not active	

		if (bitRead(COM_reg, 0) == true) { //find active program
			if (bitRead(PRG_reg[pa], 0) == true) COM_psac(pa);
			pa++;
			if (pa > 9) { //alleen programmaas scannen die door active gestart kunnen worden
				pa = 0;
				COM_reg &= ~(1 << 0); //reset bit 0, next cycle not active
			}
		}
		else { //find not-active program
			if (bitRead(PRG_reg[pna], 1) == true & PRG_hr[pna] == mt_hr & PRG_min[pna] == mt_min) COM_psmt(pna);
			COM_reg |= (1 << 0); //next cycle active
			pna++;
			if (pna > 30)pna = 0;
		}
	}
}
void COM_psac(byte pn) { //active programs
	switch (pn) {
	case 0://schakel daglicht in of uit
		PRG_dl();
		break;
	case 1:
		PRG_lightning();
		break;
	}
}
void COM_psmt(byte pn) { //Start program by model time
	switch (pn) {
	case 0:
		//daglicht start via modeltijd....wake up for new sunset/sunrise cycle	
		if (mt_hr == mt_zonop) {
			//Serial.println(F("Sunrise"));
			COM_reg &= ~(1 << 2);
			//dagnacht = false;
		}
		else {
			//dagnacht = true;
			COM_reg |= (1 << 2);
			//Serial.println(F("Sunset"));
		}
		PRG_reg[0] |= (1 << 0); //enable active
		PRG_reg[0] &= ~(1 << 1); //disable modeltime start
		break;
	case 1: //lightning wake up in modeltime
		PRG_lightning();
		break;

	case 2: //lasser 1
		FX_mtstart(2);
		break;

	case 3: //lasser 2
		FX_mtstart(3);
		break;
	case 4: //fire and glow
		FX_mtstart(4);
		break;
	case 6: //TV
		FX_mtstart(6);
		break;
	case 9: //nachtomschakeling verkeerslichten
		/*
		bit0=enable not used
		bit1=model time start
		bit2=enable nachtstand
		bit3=enable blink traffic lights
		*/
		if (bitRead(PRG_reg[9], 2) == true) {
			PRG_reg[9] &= ~(1 << 2);
			PRG_hr[9] = mt_zonop - 2;
			PRG_reg[9] |= (1 << 3); //enable night blink
			PRG_reg[27] &= ~(1 << 1);//disable modeltime start traffic 1
			PRG_reg[28] &= ~(1 << 1); //idem traffic 2
			PRG_reg[29] &= ~(1 << 1);//idm traffic 3
			for (byte i = 17; i < 27; i++) {
				LED_setPix(i, 0, 0, 0); //black all traffic lights
			}
			//Serial.println(PRG_hr[9]);
			//Serial.println(PRG_min[9]);
		}
		else {
			PRG_reg[9] |= (1 << 2);
			PRG_hr[9] = 1;
			PRG_reg[9] &= ~(1 << 3); //disable blink nachtstand 
			PRG_reg[27] = B00000010; //enable modeltime start traffic 1
			PRG_reg[28] = B00000010;
			PRG_reg[29] = B00000010;

			interval(27, 1, 0);
			interval(28, 2, 0);
			interval(29, 3, 0);
		}

		break;

	case 10:
		PRG_huis(pn, 0, 1);
		break;
	case 11:
		PRG_huis(pn, 1, 1);
		break;
	case 12:
		PRG_huis(pn, 2, 2);
		break;
	case 13:
		PRG_huis(pn, 3, 1);
		break;
	case 14:
		PRG_huis(pn, 4, 3);
		break;
	case 15:
		PRG_huis(pn, 5, 4);
		break;
	case 16: //hh1-1
		PRG_huis(pn, 6, 5);
		break;
	case 17: //hh1-2
		PRG_huis(pn, 7, 1);
		break;
	case 18: //hh2-1
		PRG_huis(pn, 8, 5);
		break;
	case 19: //hh2-2
		PRG_huis(pn, 9, 1);
		break;
	case 20: //seinhuis
		PRG_huis(pn, 10, 6);
		break;
	case 21: //Fabriek
		PRG_huis(pn, 11, 7);
		break;
	case 22: //Station1
		PRG_huis(pn, 12, 8);
		break;
	case 23: //station2
		PRG_huis(pn, 13, 9);
		break;
	case 24: //xtra1 winkel?
		PRG_huis(pn, 14, 10);
		break;
	case 25:
		PRG_huis(pn, 15, 11);
		break;
	case 26:
		PRG_huis(pn, 16, 12);
		break;

	case 27: //traffic1
		PRG_traffic1();
		break;
	case 28:
		PRG_traffic2();
		break;
	case 29:
		PRG_traffic3();
		break;
	case 30: //straatverlichting 
		PRG_straat();
		break;

	}

}
void SW_com() {
	byte temp;
	static byte cnt;
	//if (millis() - Sw_time > 100) { //100ms 
	//	Sw_time = millis();
	SW_new = PINC;
	SW_new = SW_new << 4;
	SW_new = SW_new >> 4;
	SW_change = SW_new ^ SW_old;

	if (bitRead(GPIOR0, 5) == true) LED_program();
	//timer zolang bit7 in sw_reg true

	if (bitRead(SW_reg, 7) == true) {
		//bit0-3 =teller, to spare 1 byte of memory
		cnt++;
		if (cnt > 50) {

			prgedit = 0;
			GPIOR0 |= (1 << 6); //disable modeltime clock
			SW_reg |= (1 << 6); //Main Programming enabled
			SW_reg &= ~(1 << 5); //disable pixel programming (new 16nov)
			SW_programs();
			cnt = 0;
		}
	}
	if (SW_change > 0) {
		if (bitRead(SW_reg, 7) == false) {

			for (byte i = 0; i < 4; i++) {

				if (bitRead(SW_change, i) == true & (bitRead(SW_new, i) == false)) SW_div(i); //calls if switch is pressed		
				if (bitRead(SW_reg, 7) == true)i = 5; //exit loop to prevent double run SW_both
			}
		}
		else { //sw0+sw1
			//chech bit0 and bit1 in changed
			SW_change = SW_change << 6;

			if (SW_change > 0) { //sw0 or sw1 = released start normal operation
				//Serial.println("Losgelaten");
				SW_reg &= ~(1 << 7);
				cnt = 0;
			}
		}
	}
	SW_old = SW_new;
	//}
}
void COM_rt() { //reset timers
//resets all building vl pixel parameters, after power-up and manual switching between day and night.
//Serial.println(F("COM_rt()"));

	if (bitRead(COM_reg, 2) == false) {
		mt_hr = mt_zonop; //set modeltijd 
		mt_min = 0;
	}
	else {
		mt_hr = mt_zononder; //set modeltijd
		mt_min = 0;
	}
	//Serial.println(mt_hr);
	//Serial.println(mt_min);	   
	//alle pixels in VL uitzetten
	for (byte i = 0; i < 32; i++) { //BELANGRIJK AANTAL PIXELS GELIJK AAN DECLARATIE
		if (i < 32) {
			led_vl[i] = 0x000000;
		}
		else {
			led_fx[i - 32] = 0x000000;
		}
	}

	GPIOR0 &= ~(1 << 6); //enable modeltime clock
	// special programs instellingen	
	//start traffic1 (groot verkeersplein)			
	PRG_reg[27] = B00000010; //enable modeltime start and initialise register
	PRG_hr[27] = mt_hr;
	PRG_min[27] = 3;
	//start traffic 2 (enkelvoudig kruispunt)
	PRG_reg[28] = B00000010;
	PRG_hr[28] = mt_hr;
	PRG_min[28] = 01;
	//start traffic 3 (voetganger oversteek)
	PRG_reg[29] = B00000010;
	PRG_hr[29] = mt_hr;
	PRG_min[29] = 00;

	//Start program voor verkeerslichten nachtregeling
	PRG_reg[9] = B00000110; //enable modeltime start nachtschakeling verkeerslichten
	PRG_hr[9] = 1; //uitschakelen verkeersregeling om 1 uur in de nacht
	PRG_min[9] = 0;

	for (byte i = 32; i < 37; i++) {
		LED_setPix(i, 0, 0, 0); //kill pixels on outputs FX
	}
	for (byte i = 2; i < 10; i++) {
		PRG_reg[i] &= ~(1 << 0); //kill running fx programs
		switch (i) {
		case 2: //start tijd lasser 1							
			interval(2, random(30, 90), 1);
			break;
		case 3: //starttijd lasser 2
			interval(3, random(30, 90), 1);
			break;
		case 4: //starttijd fire en glow
			interval(4, random(10, 120), 2);
			break;
		case 6: //start TV
			if (bitRead(COM_reg, 2) == true) {
				interval(6, 10, 2);
			}
			else {
				interval(6, random(510, 780), 1);
			}
			break;
		}
	}
	//PRG_huis programma's starttijd instellen
	for (byte i = 10; i < 27; i++) { //27	
		PRG_hr[i] = mt_zononder;
		PRG_min[i] = random(0, 30);
		PRG_reg[i] = 2;
	}
	//set daglicht starters, voor eerste start na powerup
	LED_setLed(10, 1, 200);
	LED_setLed(11, 1, 200);
	LED_setLed(12, 1, 200);
	LED_setLed(13, 2, 200);
	LED_setLed(14, 1, 200);
	LED_setLed(15, 1, 200);
	LED_setLed(15, 0, 200);
	LED_setLed(16, 1, 200);

	//PRG_reg[30] straat en openbare verlichting start instellen
	if (bitRead(COM_reg, 2) == true) { //nacht
		//start in fase 2
		PRG_reg[30] = B00100010;
		PRG_hr[30] = mt_zononder;
		PRG_min[30] = 10;
	}
	else { //dag
		//start in fase 0
		PRG_reg[30] = B00000010;
		PRG_hr[30] = mt_zononder - 1;
		PRG_min[30] = 45;
	}
}
void COM_tday() {
	mt = (EEPROM.read(501) * 1000) / 24; //calculate clock interval modeltime duration
}
void COM_SrS() {
	byte factor;
	switch (led_al) {
	case 30:
		factor = 100;
		break;
	case 60:
		factor = 80;
		break;
	case 90:
		factor = 60;
		break;
	case 120:
		factor = 38;
		break;
	case 150:
		factor = 25;
		break;
	case 180:
		factor = 20;
		break;
	case 210:
		factor = 15;
		break;
	case 240:
		factor = 10;
		break;
	}
	SrS = Srspeed * 2 * factor * 24; //speed of sunrise
	Pmax = led_al - 1;
}
void SW_programs() {
	//loads program to be edited	
	//reset parameters of programs
	GPIOR0 &= ~(1 << 5); //reset DCC mode
	LED_on(0);
	switch (prgedit) {
	case 0:
		GPIOR0 |= (1 << 5); //enter DCC receive mode, see APP_COM 
		DSP_txt(0);
		break;
	case 1:  //DCC mode
		DSP_txt(13);
		break;
	case 2: //speed of sunrise and sunset effect
		DSP_txt(1);
		break;
	case 3: //aantal pixels in daglicht
		DSP_txt(6);
		break;
	case 4: //Duration modeltime day in minutes
		DSP_txt(8);
		break;
	case 5: //weather type
		DSP_txt(11);
		break;
	case 6: //hour start sunrise
		DSP_txt(9);
		break;
	case 7: //hour start sunset
		DSP_txt(10);
		break;
	case 8: //lasser 1 enable modeltijd 		
		DSP_txt(2);
		LED_on(bitRead(PRG_reg[2], 1));
		break;
	case 9: //lasser 2 enable modeltijd
		DSP_txt(3);
		LED_on(bitRead(PRG_reg[3], 1));
		break;
	case 10: //fire and glow enable model time start
		DSP_txt(4);
		LED_on(bitRead(PRG_reg[4], 1));
		break;
	case 11: //tv enable modeltime start
		DSP_txt(5);
		LED_on(bitRead(PRG_reg[6], 1)); //program 6, 5 = fire2
		break;
	case 12: //flash, lightning enable modeltime start
		DSP_txt(12);
		LED_on(bitRead(COM_set, 0));
		break;
	default:
		DSP_txt(100);
		break;
	}
}
void SW_div(byte sw) {
	//dag nacht enable mt clock, decrement program
	   /*
	   Serial.println(sw);
	   Serial.println(led_al);
	   Serial.println(Srspeed);
	   Serial.println(tday);
	   Serial.println();
	   */
	if (bitRead(SW_new, 1) == false & bitRead(SW_new, 0) == false) { //both switches pressed
		SW_both();
	}
	else {
		//3 opties wat de switch kan gaan doen
		if (bitRead(SW_reg, 6) == true) { //main programming
			SW_mainprg(sw);
		}
		else { //pixel program
			if (bitRead(SW_reg, 5) == true) {
				SW_pixprg(sw);
			}
			else { //normaal operation
				SW_normal(sw);
			}
		}
	}
}
void SW_normal(byte sw) {
	//Serial.println("normal");
	switch (sw) {
	case 0:
		COM_reg ^= (1 << 2); //toggle day/night
		PRG_reg[0] |= (1 << 0); //starts program 2 PRG_dl()
		PRG_reg[0] |= (1 << 7); //flag for changed
		COM_rt();
		break;
	case 1:
		dld_com(0);
		break;
	case 2:

		break;
	case 3:
		PRG_lightning();
		break;

	}
}
void SW_pixprg(byte sw) {
	//Serial.println("pixprg");
	switch (sw) {
	case 0:
		SW_count--;
		if (SW_count > 47) SW_count = 47;
		break;
	case 1:
		SW_count++;
		if (SW_count > 47) SW_count = 0;

		break;
	case 2:
		led_vlap[SW_count] = led_vlap[SW_count] - 1;
		if (led_vlap[SW_count] > 47)led_vlap[SW_count] = 47;
		break;
	case 3:
		led_vlap[SW_count] = led_vlap[SW_count] + 1;
		if (led_vlap[SW_count] > 47)led_vlap[SW_count] = 0;
		break;
	}
	DSP_pix();
}
void SW_mainprg(byte sw) {
	//Serial.println(sw);
	switch (sw) {
	case 0: //switch prg-
		prgedit--;
		SW_programs();
		break;
	case 1: //switch prg +
		prgedit++;
		SW_programs();
		break;
	case 2: //switch value -
		switch (prgedit) {
		case 1: //DCC mode dec
			if (COM_DCCmode > 0) {
				COM_DCCmode--;
				DSP_txt(13);
			}
			break;
		case 2: //speed sunset,sunrise
			if (Srspeed >= 0) {
				Srspeed--;
				DSP_txt(1);
			}
			break;

		case 3: //qty pixels in dl
			if (led_al > 30)led_al = led_al - 30;
			DSP_txt(6);
			break;
		case 4: //duration modeltime
			if (tday > 10)tday--;
			DSP_txt(8);
			break;
		case 5: //Weather type
			if (DL_wt > 0) DL_wt--;
			DSP_txt(11);
			break;
		case 6:
			if (mt_zonop > 5) mt_zonop--;
			DSP_txt(9);
			break;
		case 7:
			if (mt_zononder > 18)mt_zononder--;
			DSP_txt(10);
			break;
		case 8: //lasser 1 off
			PRG_reg[2] &= ~(1 << 1); //reset model time start
			COM_set &= ~(1 << 1);
			LED_on(0);
			break;
		case 9: //lasser 2 off
			PRG_reg[3] &= ~(1 << 1); //reset model time start
			COM_set &= ~(1 << 2);
			LED_on(0);
			break;
		case 10: //fire and glow
			PRG_reg[4] &= ~(1 << 1); //reset modeltime start fire
			PRG_reg[5] &= ~(1 << 1); //reset modeltime start glow
			COM_set &= ~(1 << 3); //reset bit in register
			LED_on(0);
			break;
		case 11: //tv
			PRG_reg[6] &= ~(1 << 1); //reset modeltime start tv
			COM_set &= ~(1 << 4); //reset bit in register
			LED_on(0);
			break;
		case 12: //lightning effect starts on modeltime
			COM_set &= ~(1 << 0);
			LED_on(0);
			break;

		}
		break;

	case 3: //switch value +
		switch (prgedit) {
		case 1: //DCC mode inc
			if (COM_DCCmode < 2) {
				COM_DCCmode++;
				DSP_txt(13);
			}
			break;
		case 2:
			if (Srspeed < 10) {
				Srspeed++;
				DSP_txt(1);
			}
			break;
		case 3: //qty pixels in DL
			if (led_al < 211)led_al = led_al + 30;
			DSP_txt(6);
			break;
		case 4: //duration modeltime
			if (tday < 60)tday++;
			DSP_txt(8);
			break;
		case 5:
			if (DL_wt < 4)DL_wt++;
			DSP_txt(11);
			break;
		case 6:
			if (mt_zonop < 10)mt_zonop++;
			DSP_txt(9);
			break;
		case 7:
			if (mt_zononder < 22) mt_zononder++;
			DSP_txt(10);
			break;
		case 8: //lasser 1 on
			PRG_reg[2] |= (1 << 1); //set model time start
			COM_set |= (1 << 1);
			LED_on(1);
			break;
		case 9: //lasser 2 on
			PRG_reg[3] |= (1 << 1); //set model time start
			COM_set |= (1 << 2);
			LED_on(1);
			break;
		case 10: //fire on
			PRG_reg[4] |= (1 << 1); //set modeltime start fire
			PRG_reg[5] |= (1 << 1); //set modeltime start glow
			COM_set |= (1 << 3); //set bit in register
			LED_on(1);
			break;
		case 11: //tv
			PRG_reg[6] |= (1 << 1); //set modeltime start tv
			COM_set |= (1 << 4); //set bit in register
			LED_on(1);
			break;
		case 12:
			COM_set |= (1 << 0);
			LED_on(1);
			break;
		}
		break;
	}
}
void SW_both() {
	//Serial.println("alle twee");
	//twee mogelijkheden of niet in program mode dan daar inzetten, wel in programmode eruit halen
	if (bitRead(SW_reg, 6) == true | bitRead(SW_reg, 5) == true) { //program active, leave program mode
		//Serial.println("reset");
		SW_save(); //save changes
		SW_reg = 0;

		PRG_reg[0] |= (1 << 0); //activate program dl
		GPIOR0 &= ~(1 << 6); //enable modeltime clock
		GPIOR0 &= ~(1 << 5); //disable dcc mode
		GPIOR0 |= (1 << 2); //enable effects in loop
	}
	else { //no prgram active
		SW_reg |= (1 << 7);
		//hier komt afhandeling bij dubbel druk, 1x kort is start pixelprogram, of cancel programmaas. Main program wordt gestart in de timer in COM_sw
		SW_reg |= (1 << 5); //enable pixel programming
		SW_reg &= ~(1 << 6); //disable main programming
		GPIOR0 |= (1 << 6); //disable model time clock
		GPIOR0 &= ~(1 << 2); //disable effects in loop
		PRG_reg[0] &= ~(1 << 0); //set program 0 inactive
		for (byte i = 0; i < led_al; i++) { //kill all pixels in DL
			led_dl[i] = 0x000000;
		}
		LED_on(0);
		SW_count = 0;
		DSP_pix();
	}
}
void SW_save() {
	//save COM_set register
	MEM_change();

	//saves pixel/program changes
	for (byte i = 0; i < 40; i++) {

		if (EEPROM.read(i) != led_vlap[i]) {
			//Serial.println(i);
			EEPROM.write(i, led_vlap[i]);
		}
	}
}
void APP_Monitor(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//application for DCC monitor
	//dit kan straks weg????? 


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
	static byte check;
	byte mr; //***7juni van static byte naar lokaal gezet. Check bij niet werken bliksem...
	byte temp;
	//=memorie read
//Functies decoder op Mainadres
//port 1=dagnacht met  zonsopgang en ondergang effecten, start automatisch dagnacht programma, dynamisch
//port 2 = dag/nacht zonder effecten, stopt automatische dag/nacht, statisch
//port 3 ?
//port 4 Alle pixels in VL en FX on/off
//programming mainadres
	boolean ok = false;

	if (bitRead(GPIOR0, 5) == true) { //waiting for DCCadress
		COM_DCCAdres = decoder;
		EEPROM.write(100, decoder);
		LED_on(0);
		GPIOR0 &= ~(1 << 5); //enable programs
		GPIOR0 &= ~(1 << 6); //enable modeltime clock
		setup();
		channel = 5; //to prevent unwanted events, channel 5 cannot be executed
	}

	if (decoder == COM_DCCAdres) {
		if (type == false) {
			//check for same command as last, to reduce double commands
			//not sure if this works correct in all situation, when countering DCC problems check this out.

			if (channel^port^onoff != check) {

				switch (channel) {

				case 1:
					if (port == true) {
						COM_reg &= ~(1 << 2);
						//dagnacht = false;
					}
					else {
						COM_reg |= (1 << 2);//dagnacht = true;
					}
					PRG_reg[0] |= (1 << 0); //start PRG_dl
					PRG_reg[0] |= (1 << 7); //Set changed flag
					COM_rt(); //resets time depended programs	
					break;
				case 2:
					if (port == true) {
						dld_com(1);
					}
					else {
						dld_com(2);
					}
					break;
				case 3:
					break;
				case 4:
					//alle verlichting aan of uit tbv. programmeren
					byte value;
					if (port == true) {
						value = 250;
					}
					else {
						value = 0;
					}
					for (byte i = 0; i < 48; i++) {
						if (i > 39) {
							led_fx[i - 40] = CRGB(value, value, value);
						}
						else {
							led_vl[i] = CRGB(value, value, value);
						}
					}
					break;
				}
			}
			check = channel ^ port ^ onoff;
		}
		else { //CV
			switch (cv) {
			case 2: // reset eeprom
				switch (value) {
				case 10: //reset EEprom totaal, systeem reset factory reset
					MEM_reset(0, EEPROM.length());
					setup();
					break;
				}
				break;
			case 3: //speed sunset sunrise value 0-10
				if (value < 11) {
					Srspeed = value;
					ok = true;
				}
				break;
			case 4: //qty of pixels in dl
				//ok = false;
				temp = value;
				for (byte i = 0; i < 9; i++) {
					temp = temp - 30;
					if (temp == 0) {
						led_al = value;
						ok = true;
						i = 10;
					}
				}
				break;
			case 5: //duration modeltime day
				if (value > 9 & value < 61) {
					tday = value;
					ok = true;
				}
				break;

			case 6: //#502 weertype
				if (value < 5) {
					DL_wt = value;
					ok = true;
				}
				break;

			case 7: //#503 Model-tijd zonsopgang
				if (value > 4 & value < 11) {
					mt_zonop = value;
					ok = true;
				}
				break;
			case 8://#504 Model-tijd Zonsondergang
				if (value > 17 & value < 23) {
					mt_zononder = value;
					ok = true;
				}
				break;
			case 9: //lasser1 op modeltijd
				switch (value) {
				case 0:
					PRG_reg[2] &= ~(1 << 1);
					COM_set &= ~(1 << 1);
					PRG_reg[2] |= (1 << 6); //request stop					
					ok = true;
					break;
				case 1:
					PRG_reg[2] |= (1 << 1);
					COM_set |= (1 << 1);
					ok = true;
					break;
				}
				break;
			case 10: //lasser2 op modeltijd
				switch (value) {
				case 0:
					PRG_reg[3] &= ~(1 << 1);
					COM_set &= ~(1 << 2);
					PRG_reg[3] |= (1 << 6); //request stop					
					ok = true;
					break;
				case 1:
					PRG_reg[3] |= (1 << 1);
					COM_set |= (1 << 2);
					ok = true;
					break;
				}
				break;
			case 11: //modeltime start fire1,2
				switch (value) {
				case 0:
					PRG_reg[4] &= ~(1 << 1);
					COM_set &= ~(1 << 3);
					ok = true;
					break;
				case 1:
					PRG_reg[4] |= (1 << 1);
					COM_set |= (1 << 3);
					ok = true;
					break;
				}
				break;

			case 12://modeltime start TV
				switch (value) {
				case 0:
					PRG_reg[6] &= ~(1 << 1);
					COM_set &= ~(1 << 4);
					ok = true;
					break;
				case 1:
					PRG_reg[6] |= (1 << 1);
					COM_set |= (1 << 4);
					ok = true;
					break;
				}
				break;
			case 13: //lightning start on modeltime
				switch (value) {
				case 0:
					COM_set &= ~(1 << 0);
					ok = true;
					break;
				case 1:
					COM_set |= (1 << 0);
					ok = true;
					break;
				}
				break;
			}
			if (ok == true) {
				DSP_cvtxt(cv);
				MEM_change();
			}
		}
	}
}
void APP_VL(boolean type, int adres, int decoder, byte channel, boolean port, boolean onoff, int cv, int value) {
	int adresmin = (COM_DCCAdres * 4) + 1; //
	int adresmax;
	byte pixel;
	byte prg;
	byte count = 0;
	boolean infx = false;

	switch (COM_DCCmode) {
	case 0:
		adresmax = adresmin + 8;
		break;
	case 1:
		adresmax = adresmin + 56;
		break;
	case 2:
		adresmax = adresmin + 154;
		break;
	}

	if (adres >= adresmin & adres < adresmax) { // 2 in programs on/off, 40 in vl line, 8 in fx line, 

		Serial.println(adres);

		pixel = adres - adresmin;

		if (pixel > 47) infx = true;//		if (pixel > 39) infx = true;

		if (type == false) {//switch or CV

			switch (pixel) { //pixel is hier het DCC ADRES bovenop het main adres
			case 0: //program 2 Lasser 1
				if (port == true) {
					PRG_reg[2] |= (1 << 0);
					PRG_reg[2] &= ~(1 << 6); //dit zorgt voor vage 'niet willen starten' probleem
					//Serial.println("start");
				}
				else {

					PRG_reg[2] |= (1 << 6); //request for stop, program will be deactivated in program PRG_las()
					//Serial.println("stop");
				}
				break;

			case 1: //program 3 lasser 2
				if (port == true) {
					PRG_reg[3] |= (1 << 0);
					PRG_reg[3] &= ~(1 << 6);
				}
				else {
					PRG_reg[3] |= (1 << 6); //request for stop, program will be deactivated in program PRG_las()
				}
				break;
			case 2: //fire1
				if (port == true) {
					PRG_reg[4] |= (1 << 0);

				}
				else {
					PRG_reg[4] &= ~(1 << 0);
					LED_setPix(42, 0, 0, 0);
				}

				break;
			case 3://Fire glow
				if (port == true) {
					PRG_reg[5] |= (1 << 0);

				}
				else {
					PRG_reg[5] &= ~(1 << 0);
					LED_setPix(43, 0, 0, 0);
				}
				break;
			case 4: //TV
				if (port == true) {
					PRG_reg[6] |= (1 << 0);

				}
				else {
					PRG_reg[6] &= ~(1 << 0);
					LED_setPix(44, 0, 0, 0);
				}
				break;
			case 5:
				break;
			case 6:
				break;
			case 7:
				//manual start of lightning
				if (port == true) PRG_lightning();
				break;

			default:
				if (infx == false) {
					pixel = pixel + 8;
					if (port == true) {
						led_vl[pixel] = 0xFFFFFF;
					}
					else {
						led_vl[pixel] = 0x000000;
					}
				}
				else {
					pixel = pixel - 48;
					if (port == true) {
						led_fx[pixel] = 0xFFFFFF;
					}
					else {
						led_fx[pixel] = 0x000000;
					}
				}
				break;




			}
			//display
			prg = led_vlap[pixel];

			while (prg > 9) {
				count++;
				prg = prg - 10;
			}

			klok[0] = DSP_digit(prg);
			klok[1] = DSP_digit(count);
			klok[2] = DSP_digit(14);
			klok[3] = DSP_digit(channel);
		}
		else {//CV
		}
	}
}
void LED_setPix(byte output, byte r, byte g, byte b) {
	//sets new value for pixel RGB	
	for (byte i = 0; i < 48; i++) { //check all leds in this group
		if (i < 40) {
			if (led_vlap[i] == output)led_vl[i] = CRGB(r, g, b);
		}
		else {
			if (led_vlap[i] == output)led_fx[i - 40] = CRGB(r, g, b);
		}
	}
}
void LED_setLed(byte output, byte led, byte value) {
	//sets new value for 1 othe leds in a pixels
	if (led < 3) {
		for (byte i = 0; i < 48; i++) { //check all pixels
			if (led_vlap[i] == output) {
				if (i < 40) {
					switch (led) {
					case 0:
						led_vl[i].r = value;
						break;
					case 1:
						led_vl[i].g = value;
						break;
					case 2:
						led_vl[i].b = value;
						break;
					}
				}
				else {
					switch (led) {
					case 0:
						led_fx[i - 40].r = value;
						break;
					case 1:
						led_fx[i - 40].g = value;
						break;
					case 2:
						led_fx[i - 40].b = value;
						break;
					}
				}
			}
		}
	}
}
void LED_idFxLed(byte prg, byte out, byte led, byte id, byte value) {
	//moet ook gaan werken op VL leds...
	byte pix;
	for (byte i = 0; i < 48; i++) {
		if (led_vlap[i] == out) {
			if (i < 40) {
				//vl pixels
				pix = i;
				switch (led) {
				case 0:
					if (led_vl[pix].r < value) {
						led_vl[pix].r = 0;
						PRG_reg[prg] |= (1 << 3);
					}
					else {
						led_vl[pix].r = led_vl[pix].r - value;
					}
					break;
				case 1:
					if (led_vl[pix].g < value) {
						led_vl[pix].g = 0;
					}
					else {
						led_vl[pix].g = led_vl[pix].g - value;
					}
					break;
				case 2:
					if (led_vl[pix].b < value) {
						led_vl[pix].b = 0;
					}
					else {
						led_vl[pix].b = led_vl[pix].b - value;
					}
					break;
				}
			}
			else {
				//fx pixels
				pix = i - 40;
				switch (led) {
				case 0:
					if (led_fx[pix].r < value) {
						led_fx[pix].r = 0;
						PRG_reg[prg] |= (1 << 3);
					}
					else {
						led_fx[pix].r = led_fx[pix].r - value;
					}
					break;
				case 1:
					if (led_fx[pix].g < value) {
						led_fx[pix].g = 0;
					}
					else {
						led_fx[pix].g = led_fx[pix].g - value;
					}
					break;
				case 2:
					if (led_fx[pix].b < value) {
						led_fx[pix].b = 0;
					}
					else {
						led_fx[pix].b = led_fx[pix].b - value;
					}
					break;
				}
			}
		}
	}
}
void LED_program() {
	PINB |= (1 << 5);
	if (bitRead(PORTB, 5) == false) {
		PORTB |= (1 << 4);
	}
	else {
		PORTB &= ~(1 << 4);
	}
}
void LED_on(boolean onoff) {
	if (onoff == true) {
		PORTB |= (1 << 5);
		PORTB |= (1 << 4);
	}
	else {
		PORTB &= ~(1 << 5);
		PORTB &= ~(1 << 4);
	}
}
void PRG_dl() {
	//static byte temp; //kan straks weg

	static byte fxb; //hoe ver het effect naar het westen
	static byte rled[3]; //willekeurige byte
	static byte dl_sp = 1;
	static byte maxclr[3]; //max te bereiken kleuren per weer type
	static byte ledcount = 0; // welke led
	static byte dl_sc = 0; //laatste led, en teller in fase 113
	static byte teller = 0; //algemene teller	
	static byte dl_s = 0; //shift, aantal over geslagen leds dynamisch
	static byte dl_st = 0; //shift overgeslagen leds statisch
	static byte weer;
	static unsigned long dl_t = 0;
	static byte fase;

	//if (temp != fase) {
		//temp = fase;
		//Serial.println(fase);
	//}


	if (micros() - dl_t > SrS) { //speed effect COM_Srs
		dl_t = micros();
		if (bitRead(PRG_reg[0], 7) == true) {//called if day/night changes
			PRG_reg[0] &= ~(1 << 7);
			teller = 0;
			ledcount = 0;
			fase = 0;
			PRG_reg[0] &= ~(1 << 1); //disable modeltijd start
		}

		//Start switch fase
		switch (fase) {
		case 0:
			if (bitRead(COM_reg, 2) == false) { //sunrise			
				switch (DL_wt) { //0=random 1=sunny 2=clouded 3=miserable weather 4=no effects
				case 0:
					weer = random(1, 4); //0-1-2-3 (never 4)
					break;
				default:
					weer = DL_wt;
					break;
				}

				Serial.print(F("Weer: "));
				Serial.println(weer);

				fxb = random(2, (led_al / 10) * 7);
				switch (weer) {
				case 1: //zonnig
					dl_sp = 1;
					fase = 10;
					teller = 0;
					dl_st = random(2, 7); //aantal leds wat wordt overgeslagen. 
					break;
				case 2: //half bewolkt, vaak zon.
					fase = 20;
					break;
				case 3: //bewolkt weer geen kleureffecten
					fase = 40;
					dl_s = 1;   //kijk uit dubbel gebruik van deze variabele				
					maxclr[2] = random(170, 210);
					maxclr[0] = maxclr[2] - random(10, 50); maxclr[1] = maxclr[2] - random(0, 30);
					teller = 0;
					break;
				case 4:
					fase = 50;
					teller = 1;
					break;
				}
			}
			else { //sunset
				fxb = random(led_al * 4 / 10, led_al - 2);
				switch (weer) {
				case 1: //Zonnig
					fase = 100;
					dl_st = random(0, 5); //aantal leds wat wordt overgeslagen. 
					//fxb = random(al * 4 / 10, al - led_NZ);
					maxclr[0] = random(60, 120);
					maxclr[1] = maxclr[0];
					maxclr[2] = maxclr[0];
					teller = random(1, 50);
					lightningstart(3);	//	1=altijd, 2=50%, 3=33%,  4=25%	
					break;
				case 2: //regenachtige bewolkt, kans op onweer
					fase = 120;
					teller = 0;
					lightningstart(2);	//	1=altijd, 2=50%, 3=33%,  4=25%		
					break;
				case 3: //zwaar bewolkt, kans op onweer
					fase = 140;
					teller = 0;
					lightningstart(1);
					break;
				case 4:
					fase = 150;
					teller = 30;
					break;
				}
			}
			break;

		case 1://stops program
			PRG_reg[0] |= (1 << 7);
			break;
			//************WEER 1 ZONNIG
		case 10: //susnrise much effect		
			if (rled[0] == ledcount) {
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					if (ledcount < fxb) led_dl[ledcount] = CRGB(5, 0, 0); //random pixels red
				}
			}
			if (ledcount == rled[1] & led_dl[ledcount].r == 0) led_dl[ledcount] = CRGB(3, 3, 3);
			if (ledcount >= Pmax) { //>=   voor 20nov
				teller++;
				if (teller > led_al / 5) {
					teller = 0;
					fase = 13;
					maxclr[0] = random(50, 130); //te bereiken rode kleur instellen
					maxclr[1] = maxclr[0] * 4 / 10; //Geel waarde bijvoegen
				}
			}
			break;
		case 13:
			if (ledcount < fxb) {
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					wit(ledcount, 1, maxclr[0], 0, 0, true);
				}
			}
			else {
				if (ledcount == rled[2])led_dl[ledcount] = CRGB(4, 2, 0);
				if (ledcount == rled[1])led_dl[ledcount] = CRGB(2, 2, 2);
			}
			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true)) fase = 16;
			break;

		case 16:
			if (ledcount - dl_sc > dl_st) {
				dl_sc = ledcount;
				if (ledcount < fxb & led_dl[ledcount].g < maxclr[1]) {
					led_dl[ledcount].g++; //= 2;
					PRG_reg[0] &= ~(1 << 6);
				}
			}

			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true)) {
				fase = 19;
				teller = 0;
				maxclr[0] = 250; maxclr[1] = 250; maxclr[2] = 250;
			}
			if (ledcount == rled[0]) wit(rled[0], 3, maxclr[0], maxclr[1], maxclr[2], false);
			if (ledcount == rled[1])led_dl[ledcount] = CRGB(80, 35, 0);
			if (ledcount == rled[2] & led_dl[ledcount].r == 0)led_dl[ledcount] = CRGB(10, 10, 10);
			break;

		case 19:
			//white out
			if (ledcount >= Pmax) teller++;
			switch (teller) {
			case 0:
				dl_sp = 1;
				break;
			case 10:
				dl_sp = 2;
				break;
			case 25:
				dl_sp = 5;
				break;
			case 50:
				dl_sp = 10;
				break;
			}
			for (byte i = 1; i <= dl_sp; i++) {
				wit(ledcount, 1, maxclr[0], maxclr[1], maxclr[2], true);
			}
			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true)) fase = 1;
			break;
			//******************WEER 2
		case 20: //bewolkt zonnig weer. gevlekt eindresultaat?
			fase = 21;
			maxclr[0] = 245; maxclr[1] = 245; maxclr[2] = 255;
			break;
		case 21:
			if (ledcount == rled[0] | ledcount == rled[1]) {
				wit(ledcount, random(2, 10), maxclr[0], maxclr[1], maxclr[2], false);	//6
			}
			if (ledcount >= Pmax) teller++;
			if (teller > (led_al * 6 / 10)) { //7
				teller = 0;
				fase = 19;
				maxclr[0] = random(180, 250);
				maxclr[1] = maxclr[0]; maxclr[2] = maxclr[0];
			}
			else {
				if (teller > (led_al * 1 / 10) & teller < (led_al * 2 / 10)) {
					if (ledcount == rled[1] & ledcount < fxb) led_dl[ledcount] = CRGB(250, 0, 0);
				}
				if (teller > (led_al * 2 / 10) & teller < (led_al * 4 / 10)) {
					if (ledcount == rled[2]) led_dl[ledcount] = CRGB(250, random(80, 130), 0);
				}
			}
			break;
			//**********WEER 4
		case 40: //dull weather

			if (ledcount == rled[0] | ledcount == rled[1] | ledcount == rled[2]) {
				wit(ledcount, 6, maxclr[0], maxclr[1], maxclr[2], false);
			}
			if (ledcount >= Pmax) teller++;
			dl_s++;
			if (teller > (led_al / 4)) { //var calc in fase 4
				fase = 19;
				teller = 0;
				dl_s = 0;
			}
			break;

		case 50:
			//no effects from zero to full scale, not used in random weathertype
			wit(ledcount, teller, 250, 250, 250, true);
			if (ledcount >= Pmax) {
				if (teller < 20) teller = teller + 3;
				if (bitRead(PRG_reg[0], 6) == true) {
					if (teller < 6) {
						fase = 1;
					}
					else {
						teller = 1;
					}
				}
			}
			break;
			//Start sunset weather type 1
		case 100:
			zwart(ledcount, 3, 70, 70, 70, true); //reduce brightness
			if (ledcount == rled[0])led_dl[ledcount] = 0x000000;
			if (ledcount >= Pmax & bitRead(PRG_reg[0], 6) == true) {
				fase = 112;
				maxclr[1] = maxclr[0] * 3 / 10;
			}
			break;
		case 112:
			if (ledcount > fxb) {
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					zwart(ledcount, 2, 200, maxclr[1], 0, true); //yellow  pixels east
				}
				else {
					zwart(ledcount, 2, 15, 15, 15, false);
					if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
				}
			}
			else {
				zwart(ledcount, 2, 20, 20, 20, false);
				if (ledcount == rled[2])led_dl[ledcount] = CRGB(0, 0, 0);
			}
			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true))fase = 114;
			break;

		case 114:
			if (ledcount >= Pmax) {
				teller++;
			}

			if (teller < 40) { //reduce brightness set some pixels extra yellow
				zwart(ledcount, 1, 16, 16, 15, false);
				if (ledcount == rled[1])led_dl[ledcount] = CRGB(55, 20, 0);
				if (ledcount == rled[2])led_dl[ledcount] = 0x000000;
				//if (ledcount == rled[1])led_dl[ledcount] = 0x000000;
			}
			else {
				fase = 116;
			}
			break;
		case 116:
			zwart(ledcount, 1, 10, 10, 10, false);
			if (ledcount > fxb) {
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					zwart(ledcount, 1, 40, 0, 0, true); //200
				}
			}
			else
			{
				if (ledcount == rled[0])led_dl[ledcount] = 0x000000;
				if (ledcount == rled[1])led_dl[ledcount] = 0x000000;
				if (ledcount == rled[2])led_dl[ledcount] = 0x000000;
			}
			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true))fase = 200; //stop
			if (ledcount == rled[0]) led_dl[ledcount] = 0x000000;

			break;
			//Start sunset weathertype 2
		case 120:	 //sunset half bewolkt			
			maxclr[0] = random(40, 90); maxclr[1] = maxclr[0]; maxclr[2] = maxclr[0];
			fase = 122;
			break;

		case 122:
			zwart(ledcount, 3, maxclr[0], maxclr[1], maxclr[2], true);
			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true))fase = 123;
			if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
			if (ledcount == rled[2])led_dl[ledcount] = CRGB(0, 0, 0);
			break;

		case 123:
			if (ledcount >= Pmax) teller++;
			if (teller < (led_al * 4 / 10)) {
				zwart(ledcount, 2, 15, 15, 15, false);
				if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
				if (ledcount == rled[2] & ledcount > fxb)led_dl[ledcount] = CRGB(70, 25, 0);
				if (ledcount == rled[0] & teller == 5)led_dl[ledcount] = CRGB(130, random(0, 35), 0);
			}
			else {
				fase = 200;
			}
			break;

			//*****Start susnset weathertype 3
		case 140:
			zwart(ledcount, 2, 30, 30, 30, true);
			if (ledcount == rled[2])led_dl[ledcount] = CRGB(0, 0, 0);
			if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
			if ((ledcount >= Pmax) & (bitRead(PRG_reg[0], 6) == true)) {
				fase = 200;
			}
			break;

		case 150: //no effect fast black out
			zwart(ledcount, teller, 0, 0, 0, true);
			if (ledcount >= Pmax) {
				if (teller > 2)teller--;
				if (bitRead(PRG_reg[0], 6) == true) {
					if (teller == 1) {
						fase = 1;
					}
					else {
						teller = 1;
					}
				}
			}
			break;

		case 200: //black out naar nacht
			fase = 202;
			teller = 0;
			fxb = 0; //variabele is nu vrij?
			break;

		case 202://sets all leds max value 5	
			zwart(ledcount, 1, 7, 7, 7, true);
			if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
			if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
			if (ledcount == rled[2])led_dl[ledcount] = CRGB(0, 0, 0);
			if (ledcount >= Pmax & bitRead(PRG_reg[0], 6) == true)fase = 205;
			break;

		case 205:
			//random action on all pixels
			teller = random(0, 20);
			switch (teller) {
			case 0:
				zwart(ledcount, 1, 1, 1, 1, false);
				break;
			case 1:
				zwart(ledcount, 1, 1, 1, 1, false);
				break;
			case 2:
				led_dl[ledcount] = 0x000000;
				break;
			case 4:
				zwart(ledcount, 1, 1, 1, 1, false);
				break;
			}

			if (ledcount >= Pmax) {
				fase = 208;
				teller = 0;
			}
			break;

		case 208:
			//count active pixels
			if (led_dl[ledcount].r + led_dl[ledcount].g + led_dl[ledcount].b > 3)teller++;
			if (ledcount >= Pmax) {
				if (teller > led_al / 8) { //more the 10% of pixels active
					fase = 205;
				}
				else {
					fase = 210;
				}
				teller = 0;
			}
			break;
		case 210: //reduce to zero
			zwart(ledcount, 1, 1, 1, 1, true);
			if (ledcount >= Pmax & bitRead(PRG_reg[0], 6) == true)fase = 214;

			break;
		case 214: //clears colors
			dl_sp = (led_dl[ledcount].r + led_dl[ledcount].g + led_dl[ledcount].b);
			if (dl_sp == 1 | dl_sp == 2) led_dl[ledcount] = 0x010101;
			if (ledcount >= Pmax) fase = 215;
			break;

		case 215:
			if (ledcount == rled[0]) {
				for (int i = 0; i < led_al; i++) {
					dl_sp = ledcount + i;
					if (dl_sp > Pmax) dl_sp = 0;
					if (led_dl[dl_sp]) {
						led_dl[dl_sp] = 0x000000;
						i = led_al;
					}
				}
			}
			if (ledcount >= Pmax) fase = 216;
			break;
		case 216:
			if (led_dl[ledcount])teller++;
			if (ledcount >= Pmax) {
				if (teller > (led_al / 10)) {
					fase = 215;
					teller = 0;
				}
				else {
					fase = 1;
				}
			}
			break;
		}
		//end switch fase		
		ledcount++;
		if (ledcount >= led_al) {//Pmax = Led_al_1
			ledcount = 0;
			PRG_reg[0] |= (1 << 6); //set flag voor bereiken eindwaarde				
			rled[0] = random(0, Pmax);
			rled[1] = random(0, 239);
			rled[2] = random(0, 239);

			randomSeed(analogRead(A5));	//port (pinA5) must be free, not in use

			dl_sc = 0;
			dl_s++;
			if (dl_s > 5) dl_s = 0;

			if (bitRead(COM_reg, 2) == false) { //flashes the led status
				PINB |= (1 << 5);
				PORTB &= ~(1 << 4);
			}
			else {
				PINB |= (1 << 4);
				PORTB &= ~(1 << 5);
			}
		}
		//STOP PROGRAM (bit6 of register = true)
		if (bitRead(PRG_reg[0], 7) == true) { //stop this program plus init all variables
			PRG_reg[0] &= ~(1 << 7);
			ledcount = 0;
			dl_sc = 0;
			dl_s = 0;
			fase = 0;
			PRG_reg[0] &= ~(1 << 0); //disable active
			PRG_reg[0] |= (1 << 1); //enable model time start

			if (bitRead(COM_reg, 2) == false) {
				PORTB |= (1 << 5);
				PRG_hr[0] = mt_zononder;
				PRG_min[0] = 0;
			}
			else {
				PORTB |= (1 << 4);
				PRG_hr[0] = mt_zonop;
				PRG_min[0] = 0;
			}
		}
	} //time 
}
void PRG_lightning() { //Programnummer=1, lighting starts now
	byte lgt_count = led_al * 5 / 100; //% van leds als lightning
	static byte lgt_led;
	static byte lgtfase;
	static unsigned long time;
	static unsigned int interval;
	static byte atl; //aantal flitsen achter elkaar 
	static byte afl; //aantal leds in de bliksemflits
	static byte br; //helderheid
	static byte minute;
	static byte mc = 0;
	static byte mcc = 0;
	static unsigned int duur; //hoelang moet het onweren
	static unsigned int it = 0; //intensiteit van onweer

	if (minute != mt_min) {
		mc++;
		minute = mt_min;
		if (mc - mcc > 20) {
			mcc = mc;
			if (it > 2000) { //intensiveerd de bliksem. 
				it = it - 1000;
			}
			else {
				it = it + 1500;
			}
		}
	}

	if (bitRead(PRG_reg[1], 3) == false) {  //bit3 is bliksem bezig?
		//instellen bliksem
		PRG_reg[1] &= ~((1 << 1)); //disable modeltijd start    DIT toegevoegd 23/5 maar een keer starten vanuit modeltijd
		//Serial.println(F("flash"));
		PRG_reg[1] |= (1 << 3); //flag om aan te geven dat bliksem bezig is
		PRG_reg[1] |= (1 << 0);//active prg 1
		lgtfase = 0;
		time = millis();
		interval = 0;
		mc = 0;
		duur = random(60, 300); //hoe lang het moet onweren
		//Serial.print(F("Duur: "));
		//Serial.println(duur);
		it = random(6000, 12000);
	}
	else { //prg 4 =active bit 1
		if (millis() - interval > time) {
			switch (lgtfase) {
			case 0: //start
				afl = random(2, lgt_count);
				lgt_led = random(0, Pmax - afl);//bepaal startled in hemel	
				interval = random(100, it);
				atl = random(1, 5);
				lgtfase = 10;
				br = random(20, 200);
				break;
			case 10:
				for (int i = 0; i < afl; i++) {
					led_lgt[i] = led_dl[lgt_led + i]; //huidige waarde van led in geheugen stoppen
					led_dl[lgt_led + i] = CRGB(br, br, br);
				}
				interval = random(10, 200);//stoptijd instellen
				lgtfase = 20;
				GPIOR0 |= (1 << 7); //request fastled show
				break;

			case 20:
				for (int i = 0; i < afl; i++) {
					led_dl[lgt_led + i] = led_lgt[i]; //waarde led herstellen			
				}
				atl--;
				if (atl > 0) {
					interval = random(10, 50);
					lgtfase = 10;
				}
				else {
					if (mc > duur) {
						//stop prg_lightning
						PRG_reg[1] &= ~((1 << 0));
						PRG_reg[1] &= ~((1 << 3));
						PRG_reg[1] &= ~((1 << 1)); //disable modeltijd start
						//Serial.println(F("Flash end"));
					}
					else {
						lgtfase = 0;
					}
				}
				GPIOR0 |= (1 << 7); //request fastled show
				break;
			}
			time = millis();
		}
	}
}
void FX_mtstart(byte prg) {
	byte type;
	type = prg;
	if (prg == 2 | prg == 3)type = 0;
	//Serial.println(type);
	switch (type) {
	case 0:
		//Serial.println(prg);
		if (mt_hr < mt_zononder) {
			if (bitRead(PRG_reg[prg], 0) == true) {
				PRG_reg[prg] |= (1 << 6);
				interval(prg, random(15, 60), 0);
			}
			else {
				PRG_reg[prg] |= (1 << 0);
				interval(prg, random(5, 30), 0);
			}
		}
		else {
			interval(prg, random(30, 120), 1);
			PRG_reg[prg] |= (1 << 6);
		}
		break;
	case 4: //fire and glow, initial start after sunset
		if (bitRead(PRG_reg[4], 0) == true) { //active
			PRG_reg[4] &= ~(1 << 0);
			PRG_reg[5] &= ~(1 << 0);
			interval(4, random(60, 120), 2); //new start next day
			//mogelijk dat vroeg in de avond 


			//in 1 keer uit eventueel kan hier een programma komen om uit te gloeien... 29sept2018
			LED_setPix(42, 0, 0, 0);
			LED_setPix(43, 0, 0, 0);

		}
		else {
			PRG_reg[4] |= (1 << 0);
			PRG_reg[5] |= (1 << 0);
			interval(4, random(120, 420), 0); //set stop time
			//Serial.println(PRG_hr[4]);
			//Serial.println(PRG_min[4]);
		}
		break;
	case 6: //tv simulation
		if (bitRead(PRG_reg[6], 0) == true) { //active
			PRG_reg[6] &= ~(1 << 0);
			LED_setPix(44, 0, 0, 0); //kill pixel output 36
			interval(6, random(520, 820), 1); //new start next day
		}
		else {
			PRG_reg[6] |= (1 << 0);
			interval(6, random(5, 60), 3); //set stop time, after midnight
		}
		break;
	}
}
void dld_exe() {
	for (byte i = 0; i < led_al; i++) {
		if (bitRead(COM_reg, 2) == false) {
			//if(dagnacht==false){
			led_dl[i] = CRGB(250, 250, 250);
		}
		else {//nacht	
			led_dl[i] = CRGB(0, 0, 0);
		}
	}
	mt_hr = mt_zonop;
	if (bitRead(COM_reg, 2) == true) mt_hr = mt_zononder;
	//if (dagnacht == true) mt_hr = mt_zononder;
	mt_min = 0;
	COM_rt();
}
void dld_com(byte st) { //=dag/nacht 0=toggle 1=day 2=night
	switch (st) {
	case 0:
		COM_reg ^= (1 << 2); //bit3 false = day, true is night	
		break;
	case 1:
		COM_reg &= ~(1 << 2);
		break;
	case 2:
		COM_reg |= (1 << 2);
		break;
	}
	dld_exe();
	PRG_reg[0] &= ~(1 << 0);
	ClkStop();
	if (bitRead(COM_reg, 2) == false) {
		//if(dagnacht==false){
		PORTB |= (1 << 5);
		PORTB &= ~(1 << 4);

		PRG_reg[1] &= ~(1 << 0); //disable lightning
		PRG_reg[1] &= ~(1 << 3);
		PRG_reg[1] &= ~(1 << 1);
	}
	else {
		PORTB |= (1 << 4);
		PORTB &= ~(1 << 5);
	}
}
void PRG_blink() {
	/*
	called by loop every 150ms
	flashing blinking lights
	groen licht voetgangers hoofdweg traffic 1

	blink[0]
	bit0=enable out 19 green
	bit1=enable out 22 green
	bit2=enable out 19 yellow
	bit3=enable out 22 yellow
	bit4=count out 19
	bit5=count out 22
	bit6=count out 19 yellow
	bit7=count out 22 yellow

	blink[1]
	bit0 status out 19 green
	bit1 status out 22 green
	bit2 status out 19 yellow
	bit3 status out 22 yellow
	bit4 count out 25 green
	bit5 cout out 26 yellow
	bit6 status out 25 green
	bit7 status out 26 yellow


	*/

	byte pix;
	byte led;

	if (bitRead(PRG_reg[9], 3) == true) { //nachtstand verkeersregeling enabled

		pix = PRG_reg[9] >> 5; //timer snelheid knipperen gele lampen

		if (pix == 5) {
			pix = 0;
			PRG_reg[9] ^= (1 << 4);

			if (bitRead(PRG_reg[9], 4) == true) {
				//Gele stoplichten aan
				led = 250;
			}
			else {
				//gele stoplichten uit
				led = 0;
			}

			LED_setLed(17, 0, led);
			LED_setLed(18, 0, led);
			LED_setLed(20, 0, led);
			LED_setLed(21, 0, led);
			LED_setLed(23, 0, led);
			LED_setLed(24, 0, led);
			LED_setLed(25, 0, led);
			//LED_setLed(26, 0, led);
		}
		else {
			pix++;
		}
		//rebuild register
		pix = pix << 5;
		PRG_reg[9] = PRG_reg[9] << 3;
		PRG_reg[9] = PRG_reg[9] >> 3;
		PRG_reg[9] = PRG_reg[9] + pix;
	}
	else {
		for (byte i = 2; i < 4; i++) {
			//blinking  of program 29, output 26
			if (bitRead(PRG_reg[29], i) == true) { //blink active
				switch (i) {
				case 2:
					led = 2;
					break;
				case 3:
					led = 0;
					break;
				}

				if (bitRead(blink[1], i + 2) == true) {
					blink[1] &= ~(1 << i + 2);

					if (bitRead(blink[1], i + 4) == true) { //status led, bit2 of PRG-reg[28] is not used by PRG_reg[28]
						LED_setLed(26, led, 250);
						blink[1] &= ~(1 << i + 4);
					}
					else {
						LED_setLed(26, led, 0);
						blink[1] |= (1 << i + 4);
					}
				}
				else {
					blink[1] |= (1 << i + 2);
				}
			}
		}



		for (byte i = 0; i < 4; i++) {
			if (bitRead(blink[0], i) == true) {
				if (bitRead(blink[0], i + 4) == true) {
					blink[0] &= ~(1 << i + 4);

					switch (i) {
					case 0:
						pix = 19;
						led = 2;
						break;
					case 1:
						pix = 22;
						led = 2;
						break;
					case 2:
						pix = 19;
						led = 0;
						break;
					case 3:
						pix = 22;
						led = 0;
						break;
					}


					//toggle green led of pix(el) 
					blink[1] ^= (1 << i + 2);
					if (bitRead(blink[1], i + 2) == true) {
						LED_setLed(pix, led, 250);
					}
					else {
						LED_setLed(pix, led, 0);
					}
				}
				else {
					blink[0] |= (1 << i + 4);
				}
			}
		}
	}
}
void PRG_traffic1() {
	//note bit 0 = gele led, bit 1=rode led, bit2 =groene led

	//prg27 outputs: 17~22
	byte fase = PRG_reg[27] >> 2;
	switch (fase) { //max 64 steps
	case 0: //begin cycle
		for (byte i = 17; i < 23; i++) {
			LED_setPix(i, 0, 250, 0);
		}
		//nieuwe start tijd
		interval(27, 2, 0);
		fase = 1;
		break;
	case 1:
		LED_setPix(17, 0, 0, 250); //groen voor HoofdWegNoord
		interval(27, random(10, 15), 0);
		fase = 2;
		break;
	case 2:
		LED_setPix(17, 250, 0, 00); //geel voor HWN
		interval(27, 6, 0);
		fase = 3;
		break;
	case 3:
		LED_setPix(17, 0, 250, 00); //rood voor HWN
		interval(27, 6, 0);
		fase = 4;
		break;
	case 4:
		LED_setPix(19, 0, 0, 250); //groen voor voetgangers HW
		blink[0] |= (1 << 2); //enable blink yellow light warning rechtsaf slaand verkeer
		interval(27, 2, 0);
		fase = 5;
		break;
	case 5:
		LED_setPix(18, 0, 0, 250); //groen voor Zuid
		interval(27, random(10, 15), 0);
		fase = 6;
		break;
	case 6:
		blink[0] |= (1 << 0); //enable blink for green voetganger NZ
		interval(27, 4, 0);
		fase = 7;
		break;
	case 7:
		LED_setPix(18, 250, 0, 00); //geel voor Z
		interval(27, 1, 0);
		fase = 8;
		break;
	case 8:
		blink[0] &= ~(1 << 0); //disable blink voetganger NZ
		blink[0] &= ~(1 << 2); //disable blink yellow light warning rechtsaf slaand verkeer
		interval(27, 6, 0);
		LED_setPix(19, 0, 250, 00); //rood voetganger NZ

		fase = 9;
		break;
	case 9:
		LED_setPix(18, 0, 250, 0); //Rood voor Z
		interval(27, 5, 0);
		fase = 10;
		break;
	case 10:
		LED_setPix(20, 0, 0, 250); //groen voor Oost
		interval(27, random(10, 15), 0);
		fase = 11;
		break;
	case 11:
		LED_setPix(20, 250, 0, 0); //geel voor O
		interval(27, 6, 0);
		fase = 12;
		break;
	case 12:
		LED_setPix(20, 0, 250, 0); //Rood voor O
		interval(27, 4, 0);
		fase = 13;
		break;
	case 13:
		LED_setPix(22, 0, 0, 250); //groen voetganger west
		blink[0] |= (1 << 3); //enable blink yellow warning
		interval(27, 2, 0);
		fase = 14;
		break;
	case 14:
		LED_setPix(21, 0, 0, 250); //groen voor West
		interval(27, random(10, 15), 0);
		fase = 15;
		break;
	case 15:
		blink[0] |= (1 << 1); //enable blink for green voetganger OW
		interval(27, 1, 0);
		fase = 16;
		break;
	case 16:
		LED_setPix(21, 250, 0, 0); //geel voor W
		interval(27, 6, 0);
		fase = 17;
		break;
	case 17:
		blink[0] &= ~(1 << 1); //disable blink voetganger OW
		blink[0] &= ~(1 << 3); //disable blink yellow warning
		interval(27, 2, 0);
		LED_setPix(22, 0, 250, 00); //rood voetganger OW
		fase = 18;
		break;
	case 18:
		LED_setPix(21, 0, 250, 0); //Rood voor HWW
		interval(27, 3, 0);
		fase = 0;
		break;
	}
	//rebuild register
	fase = fase << 2;
	PRG_reg[27] = PRG_reg[27] << 6;
	PRG_reg[27] = PRG_reg[27] >> 6;
	PRG_reg[27] = PRG_reg[27] + fase;
}
void PRG_traffic2() {
	//note bit 0 = gele led, bit 1=rode led, bit2 =groene led
	//prg28 outputs: 23-24 bit2,bit3 not used
	byte fase = PRG_reg[28] >> 4;
	switch (fase) { //max 15 steps
	case 0:

		LED_setPix(23, 0, 250, 0);
		LED_setPix(24, 0, 250, 0);
		interval(28, 6, 0);
		fase = 1;
		break;
	case 1:
		LED_setPix(23, 0, 0, 250);
		interval(28, random(10, 15), 0);
		fase = 2;
		break;
	case 2:
		LED_setPix(23, 250, 0, 0);
		interval(28, 6, 0);
		fase = 3;
		break;
	case 3:
		LED_setPix(23, 0, 250, 0);
		interval(28, 6, 0);
		fase = 4;
		break;
	case 4:
		LED_setPix(24, 0, 0, 250);
		interval(28, random(10, 15), 0);
		fase = 5;
		break;
	case 5:
		LED_setPix(24, 250, 0, 0);
		interval(28, 6, 0);
		fase = 0;
		break;
	}
	fase = fase << 4;
	PRG_reg[28] = PRG_reg[28] << 4;
	PRG_reg[28] = PRG_reg[28] >> 4;
	PRG_reg[28] = PRG_reg[28] + fase;
}
void PRG_traffic3() { //voetgangers oversteek
	//note bit 0 = gele led, bit 1=rode led, bit2 =groene led
	//prg29 outputs: 25-26 
	byte fase = PRG_reg[29] >> 4;
	switch (fase) { //max 15 steps
	case 0:
		PRG_reg[29] &= ~(1 << 2);
		PRG_reg[29] &= ~(1 << 3);
		LED_setPix(25, 0, 250, 0);
		LED_setPix(26, 0, 250, 0);
		interval(29, 6, 0);
		fase = 1;
		break;
	case 1:
		LED_setPix(25, 0, 0, 250);
		interval(29, random(20, 30), 0);
		fase = 2;
		break;
	case 2:
		LED_setPix(25, 250, 0, 0);
		interval(29, 6, 0);
		fase = 3;
		break;
	case 3:
		LED_setPix(25, 0, 250, 0);
		interval(29, 6, 0);
		fase = 4;
		break;
	case 4:
		LED_setPix(26, 0, 0, 250); //voetgangers groen
		PRG_reg[29] |= (1 << 3); //start flash geel alarm
		interval(29, random(10, 15), 0);
		fase = 5;
		break;
	case 5: //hier voetgangers groen knipperen
		PRG_reg[29] |= (1 << 2); //start blinking green led
		interval(29, 7, 0);
		fase = 0;
		break;
	}
	fase = fase << 4;
	PRG_reg[29] = PRG_reg[29] << 4;
	PRG_reg[29] = PRG_reg[29] >> 4;
	PRG_reg[29] = PRG_reg[29] + fase;
}
void PRG_las(byte prg, byte out) {
	/*
	PRG_reg
	bit0 active
	bit1 start model time
	bit2 model time start flag
	bit3 off flag
	bit4 timer
	bit5 timer
	bit6 stop
	bit7 burn/noburn

*/

	static byte burn[2];
	byte pk;
	static byte start[2];

	switch (prg) {
	case 2:
		pk = 0;
		break;
	case 3:
		pk = 1;
		break;
	}

	start[pk] ++;
	if (start[pk] > 250) { //800
		start[pk] = random(0, 225); //750
		//Serial.println("****");
		if (bitRead(PRG_reg[prg], 6) == true) {
			PRG_reg[prg] |= (1 << 7);
		}
		else {
			PRG_reg[prg] ^= (1 << 7); //slow on off timer
			PRG_reg[prg] &= ~(1 << 5);
		}
	}

	if (bitRead(PRG_reg[prg], 7) == false) { //burn on
		burn[pk] ++;
		LED_setLed(out, 0, random(0, 250));
		LED_setLed(out, 1, random(0, 250));
		if (burn[pk] > 2) {
			PRG_reg[prg] ^= (1 << 2);

			if (bitRead(PRG_reg[prg], 2) == true) {
				burn[pk] = random(0, 3); //2 8
				LED_setLed(out, 2, 255);
			}
			else {
				LED_setPix(out, 0, 0, 0);
				burn[pk] = random(0, 3);
			}
			GPIOR0 |= (1 << 7);
		}
	}
	else { //burn off
		burn[pk] ++;
		if (burn[pk] > 3) { //5
			burn[pk] = 0;
			if (bitRead(PRG_reg[prg], 5) == false) { //start cooling down
				LED_setPix(out, 50, 50, 50);
				PRG_reg[prg] |= (1 << 5);
				PRG_reg[prg] &= ~(1 << 3);
			}
			else {
				if (bitRead(PRG_reg[prg], 3) == false) {
					LED_idFxLed(prg, out, 2, 0, 5);
					LED_idFxLed(prg, out, 1, 0, 2);
					LED_idFxLed(prg, out, 0, 0, 1);

				}
				else { //stop program
					if (bitRead(PRG_reg[prg], 6) == true) {
						PRG_reg[prg] &= ~(1 << 6);
						PRG_reg[prg] &= ~(1 << 0);
					}
				}
			}
		}
	}
}
void PRG_fireglow() {

	PINB | (1 << 5);
	//prg 5  prog(out 43
	static byte red = 0;
	static byte max;
	byte green;

	if (bitRead(PRG_reg[5], 7) == true) {
		red--;
		if (red < max) {
			PRG_reg[5] &= ~(1 << 7);
			max = random(100, 254);
		}
	}
	else {
		red = red + 2;
		if (red > max) {
			PRG_reg[5] |= (1 << 7);
			max = random(20, 100);
		}
	}
	if (red > 100) green = red - 100;
	LED_setPix(43, red, green, 0);
}
void PRG_fire() {
	//id PRG_reg bit 0 = true calls every 15ms from loop()
	//prg =4
	//PROG, output = 42

	/*
	PRG_reg
	bit0 active
	bit1 modeltime start
	bit2
	bit3
	bit4
	bit5
	bit6
	bit7 direction false = up
	*/

	static byte clr;  //
	static byte maxmin = 170;
	static byte speed = 3;
	if (bitRead(PRG_reg[4], 7) == true) {//cooling down
		clr = clr - speed;
		if (clr < maxmin) {
			maxmin = random(60, 255);
			speed = random(2, 8);
			PRG_reg[4] &= ~(1 << 7);
		}
	}
	else { //heating up
		if (clr + speed > maxmin) {
			maxmin = random(20, 150);
			speed = random(1, 8);
			PRG_reg[4] |= (1 << 7);
		}
		else {
			clr = clr + speed;
		}
	}
	//*/
	FIRE_clr(42, clr);
}
void FIRE_clr(byte out, byte clr) {
	byte px;
	if (clr < 50) {
		LED_setPix(out, clr, 0, 0);
	}
	else {
		if (clr < 100) { //red increment, green increment to yellow
			LED_setPix(out, clr, (clr - 50) * 1.5, 0);
		}
		else {
			if (clr < 240) {
				LED_setPix(out, clr, clr - 50, 0); //yellow increment
			}
			else {
				LED_setPix(out, 255, 255, 255); //full
			}
		}
	}
}
void PRG_tv() {
	/*
	prg 6
	PROG36
	*/
	static byte count;
	static byte led;
	led++;
	count++;
	if (led > 2)led = 0;
	if (count > 20) {
		count = random(0, 20);
		LED_setLed(44, led, random(0, 200));
	}
}
void PRG_huis(byte pg, byte out, byte huis) { //pg=program out=output huis=building pixel
	//max40xoutput 0-39
	PRG_reg[pg] = PRG_reg[pg] >> 2;
	byte hk = 10; //huiskamer
	byte sk = 10; //slaapkamer
	byte wc = 10;; //badkamer, wc
	byte bl = 10; //buitenlicht
	byte hl = 10; //hal
	byte dl = 10; //licht wat overdag en in avond brand (overdag)
	byte da = 10; //licht wat alleen overdag brand (Etalage)

	switch (huis) { //huis programmaas
	case 1: //h1-h2-h4
		hk = 1;
		sk = 0;
		wc = 2;
		break;
	case 2: //h3
		hk = 1;
		sk = 2;
		wc = 0;
		break;
	case 3: //h5
		hk = 1;
		sk = 0;
		hl = 2;
		break;
	case 4: //h6
		hk = 1;
		sk = 0;
		bl = 2;
		break;
	case 5:
		hk = 2;
		bl = 0;
		hl = 1;
		break;
	case 6: //seinhuis
		hk = 0;
		hl = 2;
		dl = 1;
		break;
	case 7: //fabriek
		hk = 2;
		bl = 0;
		da = 1;
		break;
	case 8: //Station1
		sk = 0;
		wc = 2;
		dl = 1;
		break;
	case 9: //Station2
		hk = 1;
		hl = 0;
		da = 2;
		break;
	case 10: //xtra1 (winkel)
		wc = 2;
		bl = 0;
		da = 1;
		break;
	case 11: //xtra2 (winkel)
		da = 1;
		dl = 0;
		wc = 2;
		break;
	case 12: //xtra3 (winkel)
		da = 1;
		hk = 0;
		wc = 2;
		break;
	}
	PRG_reg[pg] = PRG_reg[pg] + 1;

	switch (PRG_reg[pg]) { //max 63 stappen...
	case 1:
		LED_setLed(out, bl, 250);//buitenlicht aan
		interval(pg, random(2, 10), 0);
		break;
	case 2:
		LED_setLed(out, hk, 250);//huiskamer aan
		interval(pg, random(10, 20), 0);
		break;
	case 3:
		LED_setLed(out, da, 3);//daglicht zwak
		interval(pg, random(10, 20), 0);
		break;
	case 4:
		LED_setLed(out, hl, 250);//hal aan
		interval(pg, random(2, 5), 0);
		break;
	case 5:
		LED_setLed(out, wc, 250);//WC aan
		interval(pg, random(10, 20), 0);
		break;
	case 6:
		LED_setLed(out, wc, 0);// WC uit
		interval(pg, random(2, 5), 0);
		break;
	case 7:
		LED_setLed(out, hl, 0);//hal uit
		interval(pg, random(10, 60), 0);
		break;
	case 8:
		LED_setLed(out, hl, 250);//hal aan 
		interval(pg, random(2, 4), 0);
		break;
	case 9:
		LED_setLed(out, wc, 250);//wc aan
		interval(pg, random(10, 20), 0);
		break;
	case 10:
		LED_setLed(out, wc, 0);//WC uit
		interval(pg, random(2, 6), 0);
		break;
	case 11:
		LED_setLed(out, hl, 0);//hal uit
		interval(pg, random(2, 10), 3);
		break;
		//na 12 uur
	case 12: //seinhuis, daglicht uit
		LED_setLed(out, dl, 3);
		interval(pg, 2, 0);
		break;

	case 13:
		LED_setLed(out, sk, 250);//slaapkamer aan
		interval(pg, random(5, 20), 0);
		break;
	case 14:
		LED_setLed(out, wc, 250); //WC aan
		interval(pg, random(2, 15), 0);
		break;
	case 15:
		LED_setLed(out, hl, 3);//hal zwak
		interval(pg, random(2, 5), 0);
		break;
	case 16:
		LED_setLed(out, hk, random(0, 3)); //huiskamer zwak of uit
		interval(pg, random(2, 15), 0);
		break;
	case 17:
		LED_setLed(out, wc, 0); //WC uit
		interval(pg, random(5, 20), 0);
		break;
	case 18:
		LED_setLed(out, sk, 2); //slaapkamer zwak
		interval(pg, random(10, 25), 0);
		break;
	case 19:
		LED_setLed(out, sk, 0); //slaapkamer uit
		interval(pg, random(60, 180), 0);
		break;
	case 20:
		LED_setLed(out, sk, 3); //slaapkamer zwak
		interval(pg, random(2, 15), 0);
		break;
	case 21:
		LED_setLed(out, wc, 250); //WC aan
		interval(pg, random(10, 30), 0);
		break;
	case 22:
		LED_setLed(out, wc, 0); //WC uit
		interval(pg, random(2, 15), 0); //wake up sunrise
		break;
	case 23:
		LED_setLed(out, sk, 0); //slaapkamer uit
		interval(pg, random(10, 30), 1); //wake up sunrise
		break;
		//na zonsopgang
	case 24:
		LED_setLed(out, dl, 250); //daglicht aan
		interval(pg, random(2, 5), 0); //wake up sunrise
		break;
	case 25:
		LED_setLed(out, da, 250); //daglicht aan
		interval(pg, random(5, 15), 0); //wake up sunrise
		break;
	case 26:
		LED_setLed(out, sk, 250); // slaapkamer aan
		interval(pg, random(5, 15), 0);
		break;
	case 27:
		LED_setLed(out, wc, 255);//wc aan
		interval(pg, random(10, 20), 0);
		break;
	case 28:
		LED_setLed(out, wc, 0);//wc uit
		interval(pg, random(5, 12), 0);
		break;
	case 29:
		LED_setLed(out, sk, 0); //slaapkamer uit
		interval(pg, random(5, 15), 0);
		break;
	case 30:
		LED_setLed(out, hk, 255);//huiskamer aan
		interval(pg, random(15, 60), 0);
		break;
	case 31:
		LED_setLed(out, hk, 0);//huiskamer uit
		interval(pg, random(5, 15), 0);
		break;
	case 32:
		LED_setLed(out, hl, 0);//hal uit
		interval(pg, random(3, 10), 0);
		break;
	case 33:
		LED_setLed(out, bl, 0);//buitenlicht uit
		interval(pg, 10, 0);
		break;

	case 34: //last step
			 //reset startwaardes

		PRG_hr[pg] = mt_zononder;
		PRG_min[pg] = random(20, 60);



		//Serial.print(F("stp prgr:  "));
		//Serial.print(pg);
		//Serial.print(",  ");
		//Serial.print(F("#:  "));
		//Serial.println(PRG_reg[pg]);

		PRG_reg[pg] = 0;
		break;

	}
	//rebuild program register
	PRG_reg[pg] = PRG_reg[pg] << 2;
	PRG_reg[pg] |= (1 << 1);

}
void PRG_straat() {
	/*
	Straat- en openbare verlichting
	Programma 30 outputs 27 en 28
	vaste start en stop tijden 6 verschillende
		*/
	byte fase = PRG_reg[30] >> 4;
	switch (fase) { //15steps
	case 0: //init start 3/4 voor zonsondergang 20:40
		//alles nog uit hier
		LED_setLed(27, 1, 250);
		PRG_min[30] = 50;
		fase = 1;
		break;
	case 1: //20:50
		LED_setLed(27, 0, 250);
		PRG_hr[30] = mt_zononder;
		PRG_min[30] = 10;
		fase = 2;
		break;
	case 2: //2e start 21:10
		LED_setPix(27, 250, 250, 250);
		PRG_min[30] = 14;
		fase = 3;
		break;
	case 3: //21:14
		LED_setLed(28, 1, 250);
		PRG_min[30] = 50;
		fase = 4;
		break;
	case 4: //21:50
		LED_setLed(28, 0, 250);
		interval(30, 15, 0);
		fase = 5;
		break;
	case 5: //22:05
		LED_setLed(28, 2, 250);
		//alles brand, nu eerste stop
		PRG_hr[30] = 1;
		PRG_min[30] = 30;
		fase = 6;
		break;
	case 6://1:30
		LED_setLed(28, 0, 0);
		PRG_hr[30] = 2;
		PRG_min[30] = 10;
		fase = 7;
		break;
	case 7: //2:10
		LED_setLed(27, 0, 0);
		PRG_hr[30] = mt_zonop + 1;
		fase = 8;
		break;
	case 8: //8:10
		LED_setLed(27, 1, 0);
		PRG_min[30] = 30;
		fase = 9;
		break;
	case 9: //8:30
		LED_setLed(28, 1, 0);
		interval(30, 35, 0);
		fase = 10;
		break;
	case 10: //9:05
		LED_setPix(27, 0, 0, 0);
		interval(30, 30, 0);
		fase = 11;
		break;
	case 11: //9:35
		LED_setPix(28, 0, 0, 0);
		PRG_hr[30] = mt_zonop - 1;
		PRG_min[30] = 30;
		fase = 0;
		break;
	}

	//rebuild register
	fase = fase << 4;
	PRG_reg[30] = PRG_reg[30] << 4;
	PRG_reg[30] = PRG_reg[30] >> 4;
	PRG_reg[30] = PRG_reg[30] + fase;
}
void DSP_clock() {
	byte singles;
	singles = mt_min;
	byte tens = 0;
	while (singles > 9) {
		tens++;
		singles = singles - 10;
	}
	klok[0] = DSP_digit(singles);
	klok[1] = DSP_digit(tens);
	tens = 0;
	singles = mt_hr;
	while (singles > 9) {
		tens++;
		singles = singles - 10;
	}
	klok[2] = DSP_digit(singles);
	klok[3] = B00000000;
	if (tens > 0)klok[3] = DSP_digit(tens);
}
byte DSP_digit(byte dec) {
	byte digit;
	switch (dec) {
	case 0:
		digit = B11111101;
		break;
	case 1: //1 or I
		digit = B01100001;
		break;
	case 2:
		digit = B11011011;
		break;
	case 3:
		digit = B11110011;
		break;
	case 4:
		digit = B01100111;
		break;
	case 5:
		digit = B10110111;
		break;
	case 6:
		digit = B10111111;
		break;
	case 7:
		digit = B11100001;
		break;
	case 8:
		digit = B11111111;
		break;
	case 9:
		digit = B11110111;
		break;

	case 10: //black
		digit = B00000000;
		break;
	case 11: //c
		digit = B00011010;
		break;
	case 12: //d
		digit = B01111010;
		break;
	case 13: //-
		digit = B00000010;
		break;
	case 14: //P
		digit = B11001110;
		break;
	case 15: //C
		digit = B10011100;;
		break;
	case 16: //V
		digit = B01111100;
		break;
	case 17: //L
		digit = B00011100;
		break;
	case 18: //A-R
		digit = B11101110;
		break;
	case 19: //S
		digit = B10110110;
		break;
	case 20: //[]
		digit = B00111010;
		break;
	case 21: //F
		digit = B10001110;
		break;
	case 22: //E
		digit = B10011110;
		break;
	case 23: //t
		digit = B00011110;
		break;
	case 24://[
		digit = B10011100;
		break;
	case 25:  //]
		digit = B11110000;
		break;
	case 26: //black with : 
		digit = B00000001;
		break;
	case 27://y
		digit = B01110110;

		break;
	}

	return digit;
}
void DSP_bit() {
	//selects bit and starts shiftout 
	static byte bc = 0;
	switch (bc) {
	case 0:
		shft[1] = B00001110;
		break;
	case 1:
		shft[1] = B00001101;
		break;
	case 2:
		shft[1] = B00001011;
		break;
	case 3:
		shft[1] = B00000111;
		break;
	}
	shft[0] = klok[bc];
	bc++;
	if (bc > 3)bc = 0;
	DSP_shift();
}
void DSP_shift() {
	/*
	functie zet de twee bytes in de beide schuifregisters eerst shft[0] (segment) naar shift 2 daarna shft[1] (digits) naar shift 1
	pin 4 portd4 = data
	pin 5 portd5= shift clock SRCLK
	pin 6 portd6= latch clock RCLK
	*/
	byte fase = 0;
	byte bitcount = 0;
	for (int i = 0; i < 100; i++) {
		switch (fase) {
		case 0: //start new cycle
			bitcount = 0;
			fase = 1;
			break;
		case 1:
			if (bitRead(shft[bitRead(GPIOR0, 0)], bitcount) == true) {
				PORTD |= (1 << 4);
			}
			else {
				PORTD &= ~(1 << 4);
			}
			bitcount++;
			if (bitcount > 8) {
				bitcount = 0;
				GPIOR0 ^= (1 << 0); //toggle bit 0

				if (bitRead(GPIOR0, 0) == false) {
					fase = 4;  //make latch puls
				}
				else { //next byte
					fase = 1;
				}
			}
			else {
				fase = 2;
			}
			break;
		case 2:
			//shift puls maken,eerst doen op volle snelheid, eventueel kan hier een teller timer tussen komen
			PORTD &= ~(1 << 5);
			fase = 3;
			break;
		case 3:
			PORTD |= (1 << 5);
			fase = 1; //next bit
			break;
		case 4:
			//latch puls maken, eerst volle snelheid
			PORTD &= ~(1 << 6);
			fase = 5;
			break;
		case 5:
			PORTD |= (1 << 6);
			i = 101;
			break;
		}
	}
}
void DSP_txt(byte txtnum) {
	byte temp;
	byte tens = 0;
	switch (txtnum) {
	case 0: //dcc
		klok[3] = DSP_digit(10);
		klok[2] = DSP_digit(12);
		klok[1] = DSP_digit(11);
		klok[0] = DSP_digit(11);
		break;
	case 1: //Sunrise speed
		//Serial.println(Srspeed);
		klok[3] = DSP_digit(24);
		klok[2] = DSP_digit(25);
		if (Srspeed == 10) {
			klok[1] = DSP_digit(1);
			klok[0] = DSP_digit(0);
		}
		else {
			klok[1] = DSP_digit(26);
			klok[0] = DSP_digit(Srspeed);
		}
		break;
	case 2:
		klok[3] = DSP_digit(17);
		klok[2] = DSP_digit(18);
		klok[1] = DSP_digit(19);
		klok[0] = DSP_digit(1);
		break;
	case 3:
		klok[3] = DSP_digit(17);
		klok[2] = DSP_digit(18);
		klok[1] = DSP_digit(19);
		klok[0] = DSP_digit(2);
		break;
	case 4: //FIRE
		klok[3] = DSP_digit(21);
		klok[2] = DSP_digit(1);
		klok[1] = DSP_digit(18);
		klok[0] = DSP_digit(22);
		break;
	case 5: //tv
		klok[3] = DSP_digit(10);
		klok[2] = DSP_digit(23);
		klok[1] = DSP_digit(16);
		klok[0] = DSP_digit(10);
		break;
	case 6: //qty of pixels
		klok[3] = DSP_digit(14);
		temp = led_al;
		//find hundreds
		if (temp > 199) {
			klok[2] = DSP_digit(2);
			temp = temp - 200;
		}
		else {
			if (temp > 99) {
				klok[2] = DSP_digit(1);
				temp = temp - 100;
			}
			else {
				klok[2] = DSP_digit(0);
			}
		}
		//find tens
		//Serial.println(temp);
		while (temp > 9) {
			temp = temp - 10;
			tens++;
		}

		//Serial.println(tens);
		klok[1] = DSP_digit(tens);
		klok[0] = DSP_digit(temp);
		klok[1] &= ~(1 << 0); //kill dots in display

		break;
	case 7: //factory reset
		klok[3] = DSP_digit(8);
		klok[2] = DSP_digit(8);
		klok[1] = DSP_digit(8);
		klok[0] = DSP_digit(8);
		break;
	case 8: //modeltime duration
		klok[3] = DSP_digit(23);
		klok[2] = DSP_digit(27);
		temp = tday;
		//find tens
		tens = 0;
		for (byte i = 0; i < 7; i++) {
			if (temp > 9) {
				temp = temp - 10;
				tens++;
			}
			else {
				klok[1] = DSP_digit(tens);
				klok[0] = DSP_digit(temp);
				i = 10;
			}
		}
		break;
	case 9: //Set sunrise time
		klok[3] = DSP_digit(0);
		klok[2] = DSP_digit(15);
		if (mt_zonop < 10) {
			klok[1] = DSP_digit(26);
			klok[0] = DSP_digit(mt_zonop);
		}
		else {
			klok[1] = DSP_digit(1);
			klok[0] = DSP_digit(0);
		}


		break;
	case 10:
		temp = mt_zononder;
		klok[3] = DSP_digit(18);
		klok[2] = DSP_digit(16);
		if (temp > 19) {
			temp = temp - 20;
			klok[1] = DSP_digit(2);
		}
		else {
			temp = temp - 10;
			klok[1] = DSP_digit(1);
		}
		klok[0] = DSP_digit(temp);
		break;
	case 11: //Weather type dt
		klok[3] = DSP_digit(12);
		klok[2] = DSP_digit(23);
		klok[1] = DSP_digit(26);
		klok[0] = DSP_digit(DL_wt);
		break;
	case 12: //lightning txt (Flas)
		klok[3] = DSP_digit(21);
		klok[2] = DSP_digit(17);
		klok[1] = DSP_digit(18);
		klok[0] = DSP_digit(19);
		break;
	case 13: //DCC mode
		klok[3] = DSP_digit(11);
		klok[2] = DSP_digit(11);
		klok[1] = DSP_digit(26);
		klok[0] = DSP_digit(COM_DCCmode);

		break;

	case 100:
		klok[3] = DSP_digit(20);
		klok[2] = DSP_digit(20);
		klok[1] = DSP_digit(20);
		klok[0] = DSP_digit(20);
		break;
	}


}
void DSP_cvtxt(byte cv) {
	byte tens = 0;
	klok[3] = DSP_digit(15);
	klok[2] = DSP_digit(16);
	while (cv > 9) {
		cv = cv - 10;
		tens++;
	}
	klok[1] = DSP_digit(tens);
	klok[0] = DSP_digit(cv);
}
void DSP_pix() {
	byte pix;
	pix = SW_count;
	byte tens = 0;
	byte prg;

	//fills display in pixel program mode, and burn pixel
	for (byte i = 0; i < 48; i++) {
		if (i < 40) {
			led_vl[i] = 0x000000;
		}
		else {
			led_fx[i - 40] = 0x000000;
		}

	}
	if (pix < 40) {
		led_vl[pix] = 0xFFFFFF;
	}
	else {
		led_fx[pix - 40] = 0xFFFFFF;
	}

	prg = led_vlap[pix];


	while (pix > 9) {
		pix = pix - 10;
		tens++;
	}
	klok[3] = DSP_digit(tens);
	klok[2] = DSP_digit(pix);
	tens = 0;

	while (prg > 9) {
		prg = prg - 10;
		tens++;
	}
	klok[1] = DSP_digit(tens);
	klok[0] = DSP_digit(prg);
}
void interval(byte prg, unsigned int tijd, byte type) {

	//berekend de nieuwe starttijd voor een programma.
	//type 0=huidige tijd plus interval 1=zonsopgang plus interval 2=zonsondergang plus interval 3=24hr plus interval
	//check en correct lengte interval meer dan 60minuten.
	byte hr = 0;
	unsigned int minutes;
	minutes = mt_min + tijd;

	while (minutes > 59) {
		minutes = minutes - 60;
		hr++;
	}
	PRG_min[prg] = minutes;
	switch (type) {
	case 0: //from current time
		PRG_hr[prg] = mt_hr + hr;
		if (PRG_hr[prg] > 23)PRG_hr[prg] = PRG_hr[prg] - 24;

		break;
	case 1: //from sunrise		
		PRG_hr[prg] = mt_zonop + hr;
		break;

	case 2: //from sunset		
		PRG_hr[prg] = mt_zononder + hr;
		break;

	case 3: //from hr00:00			
		if (mt_hr >= 0 & mt_hr < mt_zonop) {
			PRG_hr[prg] = mt_hr + hr;
		}
		else {
			PRG_hr[prg] = hr;
		}
		break;
	}
	/*
		Serial.print(F("Prg: "));
		Serial.print(prg);
		Serial.print(",  ");
		Serial.print(F("uur:  "));
		Serial.print(PRG_hr[prg]);
		Serial.print(",  ");
		Serial.print(F("minuut:  "));
		Serial.println(PRG_min[prg]);
	*/
}
void wit(byte led, byte inc, byte mr, byte mg, byte mb, boolean stop) {

	if (led_dl[led].r + inc <= mr) {
		led_dl[led].r = led_dl[led].r + inc;
		if (stop == true) PRG_reg[0] &= ~(1 << 6);
	}


	if (led_dl[led].g + inc <= mg) {
		led_dl[led].g = led_dl[led].g + inc;
		if (stop == true) PRG_reg[0] &= ~(1 << 6);
	}
	if (led_dl[led].b + inc <= mb) {
		led_dl[led].b = led_dl[led].b + inc;
		if (stop == true)PRG_reg[0] &= ~(1 << 6);
	}
}
void zwart(byte led, byte dec, byte mr, byte mg, byte mb, boolean stop) {
	/*
	led_dl[led].r = led_dl[led].r - 1;
	led_dl[led].g = led_dl[led].g - 1;
	led_dl[led].b = led_dl[led].b - 1;
*/

	byte temp;
	temp = dec;

	if (led_dl[led].r > mr) {
		if (led_dl[led].r < dec)temp = led_dl[led].r;
		led_dl[led].r = led_dl[led].r - temp;
		temp = dec;
		if (stop == true) PRG_reg[0] &= ~(1 << 6);
	}


	if (led_dl[led].g > mg) {
		if (led_dl[led].g < dec)temp = led_dl[led].g;
		led_dl[led].g = led_dl[led].g - temp;
		if (stop == true) PRG_reg[0] &= ~(1 << 6);
	}
	if (led_dl[led].b > mb) {
		if (led_dl[led].b < dec)temp = led_dl[led].b;
		led_dl[led].b = led_dl[led].b - temp;
		if (stop == true)PRG_reg[0] &= ~(1 << 6);
	}
}
void lightningstart(byte kans) { //start lightningeffect op tijd, kans....0=nooit, 1=altijd, 2=50%, 3=33%,  4=25%	
	byte temp;
	// Serial.println(F("onweer"));

	if (bitRead(COM_set, 0) == true) { //enable start lighting by modeltime		
		//temp = random(1, kans + 1);
		if (temp == 1) {
			temp = 24 - (mt_zononder - mt_zonop); //set wakeuptime for lightning
			PRG_hr[1] = mt_zononder + random(0, temp);
			if (PRG_hr[1] > 23)PRG_hr[1] = PRG_hr[1] - 24;
			PRG_min[1] = random(0, 59);
			PRG_reg[1] |= (1 << 1); //enable modeltijd start

			//Serial.println(PRG_min[1]);
			//Serial.println(PRG_hr[1]);/
		}
	}
}
void ClkStop() {
	GPIOR0 |= (1 << 6); //disable modeltime clock
	for (byte i = 0; i < 4; i++) {
		klok[i] = DSP_digit(13);
	}
}
void loop() {

	static byte st; //timer count slow timer
	static byte ft; //fast timer
	DEK_DCCh();
	COM_ProgramAssign();


	/*
	if (bitRead(COM_reg, 3) == false){
		COM_reg |=(1<<3);
			//print instellingen
			Serial.print(led_vlap[0]);
			Serial.print("  ");
			Serial.print(led_vlap[1]);
			Serial.print("  ");
			Serial.print(mt_zonop);
			Serial.print("  ");
			Serial.print(mt_zononder);
			Serial.print("  ");
			Serial.println(DL_wt);
	}
*/



	if (millis() - FastClk > 5) { //5ms
		FastClk = millis();
		DSP_bit();
		ft++;
		if (ft > 3) {	//15ms			
			st++;
			ft = 0;
			COM_Clk();


			if (bitRead(GPIOR0, 2) == true) {
				//fx programs start, must be placed here because of timing issues			
				if (bitRead(PRG_reg[2], 0) == true) PRG_las(2, 40); //Lasser 1 
				if (bitRead(PRG_reg[3], 0) == true) PRG_las(3, 41); //Lasser 2 	
				if (bitRead(PRG_reg[4], 0) == true) PRG_fire();
				if (bitRead(PRG_reg[5], 0) == true) PRG_fireglow();
				if (bitRead(PRG_reg[6], 0) == true) PRG_tv();
			}

			if (st > 5) { //150ms
				SW_com();
				st = 0;
				PRG_blink();
				GPIOR0 |= (1 << 7);
			}
			if (bitRead(GPIOR0, 7) == true) {
				GPIOR0 &= ~(1 << 7);
				FastLED.show();
			}
		}
	}
}
