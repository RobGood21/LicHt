/*
 Name:		LicHt.ino
 Created:	12/23/2017 10:57:27 AM
 Author:	Rob Antonisse
 Version: 1.01

 LicHT is a project for full automated lighting of the model railroad, including day and night cyclus.
 Based on WS2811 adressable led control. 

 Vaste waardes:
 Pin voor daglicht pixels =8
 Pin voor verlichting pixels=7
 Aantal pixels in daglicht= 240 (local variable apix)


*/

#include <FastLED.h>
#include <eeprom.h>


//#define LPdl 8 //pin waar de daglicht ledstring op komt
//#define LPvl 7 //pin voor verlichting string

byte COM_DCCAdres= 64;// default waarde, programmable only by button 2
//programmable by CV number
//#500 CV2, value 10 reset EEPROM
byte led_al=240; //#500 CV10, aantal leds max 240
byte tday = 20; //#501 CV4, tday how long is a modeltimeday in minute 24 is good value lager dan 10 werkt het geheel niet goed
byte CV_wt = 0;//#502 CV6, Weertype
byte mt_zonop=7; //#503
byte mt_zononder = 21; //#504 
byte SrS = 25; //#510 CV5, sun rise speed in micros/40 

//tbv display
byte shft[2];


CRGB led_dl[240]; //max aantal pixels
CRGB led_vl[32];
//tbv van prg_lightning
CRGB led_lgt[12]; //max aantal leds voor bliksem, dit dient  als geheugen voor de 'oude'waarde van de led
//byte lgt_count; //aantal leds gebruikt voor bliksem
byte led_vlap[32]; //vlap=verlichting assign program, assign a program to a led
//byte led_evap[16];//evap=event assign program

byte COM_reg; //bit0 test PRG active(true)
//bit1 free
//bit2 day or night (also manual) false=day, true = night
//bit4 sunset and sunrise without effects true, false = with effects
//bit5 All stop, disable all prgrams for during programming true is disabled false=enabled programming mode

//boolean dagnacht = false; //false =dag
byte COM_set=0xFF; //default value
//bit0=Lighting on(true) or off(false)
byte PRG_reg[32]; //prg_reg 
byte SW_reg; //register booleans for the switch states
/*
bit0 = status switch A0  (dag nacht)
bit1=status switch A1 (clock/ program)
*/

byte PRG_min[32]; //Time next active minute
byte PRG_hr[32]; //Time next actice hour

unsigned long Clk;
unsigned int mt; //modeltime minute

//5-6-2018 variabelen volatile gemaakt, onduidelijk waarom minutes 0-7 niet werden gestart... later wel zonder aanpassing..? 
volatile byte mt_min; //modeltimeclock minutes 
volatile byte mt_hr; //modeltimeclock hours

//basic adres, adres Daylight decoder DL +1 
//VL (verlichting) decoder 16 adresses higher
//EV (events) decoder 16adresses higher

volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts

byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[6]; //registerbyte for 12 command buffers (terug naar 6 buffers)
					 //bit7= free (false) bit0= CV(true)
byte DEK_Buf0[6];
byte DEK_Buf1[6];
byte DEK_Buf2[6];
byte DEK_Buf3[6];
byte DEK_Buf4[6];
byte DEK_Buf5[6];

void setup() {
	//test mode

	delay(350);
	Serial.begin(9600);
	Serial.println(F("void setup"));
	PORTD = 0; //reset portD
	
	MEM_read();
	
	DDRB |= (1 << 5);	//pin13
	DDRB |= (1 << 4);  //Pin12 als output
	DDRB |= (1 << 3); //PIN11 as output

	DDRC &= ~(1 << 0); //set PINA0 as input
	DDRC &= ~(1 << 1); //set PINA1 as input

	//tbv clock display
	DDRD |= (1 << 4); //serial data pin as output (PIN4)
	DDRD |= (1 << 5); //shift clock output(PIN5)
	DDRD |= (1 << 6);//Shift latch output(PIN6)
	GPIOR0 = 0; //general purpose register used for flags and booleans


	Clk = millis();	
	//FastLED.setMaxPowerInVoltsAndMilliamps(5, 7000);	
	
	//instellen modeltijd bij power up
	mt_hr = mt_zonop;
	mt_min = 0;

	randomSeed(analogRead(0));

	//FastLED.setMaxPowerInVoltsAndMilliamps(5, 8000);	
	//FastLED.setBrightness (255); //beter niet

	//DeKoder part, interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1	
	setup_prg();
}
void setup_prg() {
	//init of the 32 programs	
	for (byte i = 0; i < 32; i++) {
		PRG_reg[i] = B00000000; //init program registers
		//singles
		switch (i) {
		case 2:
			//wake up program daylight.
			PRG_reg[2] |= (1 << 0);
			PRG_reg[2] |= (1 << 7);
			break;
		}
	}
	//series
	BLD_reset(); //resets the building model time depended programs
}
void MEM_read() {
	byte apix = 240;
	//merk op volgorde van variable instellen is belangrijk, eerst COM_DCCadres en COM_DCCadres instellingen
	if (EEPROM.read(0) != 0xFF) COM_DCCAdres = EEPROM.read(0);
	if (EEPROM.read(400) != 0xFF) COM_set = EEPROM.read(400); //#400 COM_set register
	
	if (EEPROM.read(500) != 0xFF) led_al = EEPROM.read(500); //aantal pixels in Daglicht
	FastLED.addLeds<NEOPIXEL, 8>(led_dl, led_al);//create strip of 32leds on pin7 'verlichting' Xx leds on pin 8 'Daglicht'
	FastLED.addLeds<NEOPIXEL, 7>(led_vl, 32);//create strip of 32leds on pin7 'verlichting'
	
	if (EEPROM.read(501) != 0xFF) tday = EEPROM.read(501); //duur van model-tijd dag instellen
	mt = (tday * 1000) / 24;	

	if (EEPROM.read(502) != 0xFF) CV_wt = EEPROM.read(502); //Weertype 0=random 1=zon 2=halfbewolkt 3=bewolkt
	if (EEPROM.read(503) != 0xFF) mt_zonop = EEPROM.read(503); //Model-tijd zonsopgang tijd
	if (EEPROM.read(504) != 0xFF) mt_zononder = EEPROM.read(504); //Model-tijd Zonsondergang tijd
	
	if (EEPROM.read(510) != 0xFF)SrS = EEPROM.read(510); //speed of sunrise
	//lgt_count = led_al * 5 / 100; //% van leds als lightning

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

	//Default instelling toewijzing pixel aan programmanummer
	//Programs VL:          program number		program outputs			assigned leds
	//huis1					10					1						0	
	//PRG_traffic			12					niet bepaald						niet bepaald

	//dus pixel 0 krijgt output 1
	for (int i = 0; i < 32; i++) {
		led_vlap[i] = i + 1;
	}


	//geheugen toewijzing pixel aan programma nummer nu max 32 pixels
	for (int i = 1; i < 33; i++) {
		if (EEPROM.read(i) < 0xFF) led_vlap[i] = EEPROM.read(i);
	}
}
void MEM_reset(int start,int aantal) {
	//resets EEprom to 0xFF from start to start+64
	//Reloads led assign to predifined values
	for (int i = start; i < start + aantal; i++) {
		if (EEPROM.read(i) < 0xFF)EEPROM.write(i, 0xFF);
	}
MEM_read();
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
void DEK_BitRX() { //new version
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0;
	//static byte n = 0; //7juni weggehaald, functie niet duidelijk
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
	APP_FX(type, adres, decoder, channel, port, onoff, cv, value);
	APP_VL(type, adres, decoder, channel, port, onoff, cv, value);
}
void COM_Clk() {	
	if (millis() - Clk > mt & bitRead(COM_reg,5)==false & bitRead(COM_reg,4)==false) { //1 minute in modelrailroad time
			//modelroad time. 1 day standard 24 minutes. (can be updated by CV, DCC or calculation faster of slower)
			//minium timing is an hour modelroad time, faster events will be done on real time
		Clk = millis();
		mt_min ++;

		/*
		if (bitRead(PRG_reg[2], 1) == true) {
			 // toggle led on pin 11, toont dag nacht automatisch
		}
		else {
			if (bitRead(PORTB,3)==true) PORTB &= ~(1 << 3);
		}
		*/

		//blauwe led altijd als clock loopt...
		PINB |= (1 << 3);


		if (mt_min > 59) {
			mt_min = 0;
			mt_hr++;


			if (mt_hr > 23) {
				mt_hr = 0;
			}
			Serial.print(F("MT uur:  "));
			Serial.println(mt_hr);
		}		
	}
}
void COM_ProgramAssign() {
	//plays, assigns programs only 1 every cycle 
	//first 64 active programs 1by1 then only 1 not active programs, then 64 active and so on. 	
	//after start all programs must be passed once
	static byte pa; //program active
	static byte pna;//program not active	
	//static byte temp;

	if (bitRead(COM_reg, 0) == true) { //find active program
		if (bitRead(PRG_reg[pa], 0) == true) COM_ps(pa);
		pa ++;
		if (pa > 31) {
			pa = 0;
			COM_reg &= ~(1 << 0); //reset bit 0, next cycle not active
		}
	}
	else { //find not-active program
/*

		if (temp != mt_min) {
			//Serial.println(mt_min);
			temp = mt_min;
		}
*/

		if (bitRead(PRG_reg[pna], 0) == false & bitRead(PRG_reg[pna], 1) == true & PRG_hr[pna] == mt_hr & PRG_min[pna] == mt_min) {	

			

		//schakelen op modeltijd, eventuele initialisering van het te starten programma
			switch (pna) {
			case 2:
				//daglicht start via modeltijd....wake up for new sunset/sunrise cycle	
				if (mt_hr == mt_zonop) {
					Serial.println(F("Sunrise"));
					COM_reg &= ~(1 << 2);
					//dagnacht = false;
				}
				else {		
					//dagnacht = true;
					COM_reg |= (1 << 2);
					Serial.println(F("Sunset"));
				}
				PRG_reg[2] |= (1 << 0); //enable active
				PRG_reg[2]|= (1 << 7);
				//nieuw
				//PRG_reg[2] &= ~(1 << 1); 
				break;

			default:			
			Serial.print(F(": "));
			Serial.print(pna);
			Serial.print(", ");
			COM_ps(pna);
			break;
			}
		}
		COM_reg |= (1 << 0); //next cycle active
		pna ++;
		if (pna > 31)pna = 0;
	}
}
void COM_ps(byte pn) { //ps=program switch
	//total, max 64 programs
	//1-10 daylight and weather 11/30 lighting houses and streetlights  31 > no idea yet
	//Serial.println(bitRead(PRG_reg[3], 0));

	if (bitRead(COM_reg, 5) == false & bitRead(COM_reg,4) ==false) {	//enable normal mode
		switch (pn) {
		case 2: //schakel daglicht in of uit
			PRG_dl(pn);
			break;
		case 4:
			PRG_lightning();
			break;

/*			
					case 6:
						//PRG_las();
						break;
					case 9:
						//PRG_traffic(pn);
						break;
			*/	
		case 10:
			//Serial.print("*");
			PRG_huis(pn, 1, 1);
			break;		
			
		case 11:
			PRG_huis(pn, 2, 1);
			break;
		case 12:
			PRG_huis(pn, 3, 2);
			break;
		case 13:
			PRG_huis(pn, 4, 1);
			break;
		case 14:
			PRG_huis(pn, 5, 3);
			break;
		case 15:
			PRG_huis(pn, 6, 4);
			break;
		case 16: //hh1-1
			PRG_huis(pn, 7, 5);
			break;
		case 17: //hh1-2
			PRG_huis(pn, 8, 1);
			break;
		case 18: //hh2-1
			PRG_huis(pn, 9, 5);
			break;
		case 19: //hh2-2
			PRG_huis(pn, 10, 1);
			break;	
		}
	}
}
void COM_switch() {
	static long Sw_time = 0;
	static unsigned int count = 0;
	static unsigned int pc = 0; //program count
	if (millis() - Sw_time >50) { //every xxms
		Sw_time = millis(); //reset counter
		//prgram switch op A1
		//tbv leds blinking in programmode, waiting for DCC adres

//************tijdelijk??? starten van shiftfunctie
		GPIOR0 |= (1 << 1); //enable void DSP_shift





		if (bitRead(COM_reg, 5) == true)LED_program();
		if (bitRead(PINC, 1) == false) { //switch A1 pressed	
			if (bitRead(SW_reg, 1) == false) {
				SW_reg |= (1 << 1);				
				if (bitRead(COM_reg, 5) == true) {
					COM_reg &=~(1 << 5);
					LED_off();
				}
			}
			else {
				pc++;
				if (pc > 40 & bitRead(COM_reg,5)==false) { //> 2 seconds
					COM_reg |= (1 << 5); //disable all programs
					LED_off();		
				}
				if (pc > 250) {
					COM_reg &= ~(1 << 5);
					LED_off();
					pc = 0;
					Serial.println("factory reset");
					MEM_reset(0,EEPROM.length());
				}
			}
		}
		else { //switch A1 not pressed
			SW_reg &= ~(1 << 1);	
			pc = 0;
		}
		//test switch A0 dag/nacht
		if (bitRead(PINC, 0) == false) { //switch A0 pressed			
			if (bitRead(SW_reg, 0) == false) {
					SW_reg |= (1 << 0); //set switchstate
			}
			else { //switchstate true so button was allready pressed, wait for 2 seconds
				count++;	

				if (count > 40 & bitRead(COM_reg,1)==false){   // bit 1 blokkeert omschakeling tot nadat knop is losgelaten. 
						COM_reg |= (1 << 1);
					dld_com(0);
				}
			}
		}
		else { //button released			
			if (bitRead(SW_reg, 0) == true) {
				if (bitRead(COM_reg,1) ==false) {
					PRG_reg[2] |= (1 << 0); //starts program 2 PRG_dl()
					PRG_reg[2] |= (1 << 7); //flag for changed
					BLD_reset(); //reset tijd afhankelijke programmaas, alleen bij handschakeling, anders in PRG_dl zetten
					//COM_reg ^= (1 << 2); //bit2 false = day, true is night

					COM_reg ^=(1 << 2);


					/*
					if (dagnacht == true) {
						dagnacht = false;
					}
					else {
						dagnacht = true;
					}
					*/

					PORTB &= ~(1 << 4);
					PORTB &= ~(1 << 5);
				}
				count = 0;
				COM_reg &= ~(1 << 1);//dld = false;
				SW_reg &= ~(1 << 0); //reset switch state
			}
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
		//=memorie read
	//Functies decoder op Mainadres
	//port 1=dagnacht met  zonsopgang en ondergang effecten, start automatisch dagnacht programma, dynamisch
	//port 2 = dag/nacht zonder effecten, stopt automatische dag/nacht, statisch
	//port 3 ?
	//port 4 ?
	//programming mainadres
	if (bitRead(COM_reg, 5) == true) { //waiting for DCCadress
		COM_DCCAdres = decoder;
		EEPROM.write(0, decoder);
		LED_off();
		COM_reg &= ~(1 << 5);
		MEM_read();
	}

	if (decoder==COM_DCCAdres) {
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
					COM_reg |= (1 << 2);
					//dagnacht = true;
				}
				PRG_reg[2] |= (1 << 0); //start PRG_dl
				PRG_reg[2] |= (1 << 7); //Set changed flag
				BLD_reset(); //resets time depended programs	
				break;
			case 2:
				if (port == true) {
					dld_com(1);
					Serial.println(F("Aan"));
				}
				else {
					Serial.println(F("uit"));
					dld_com(2);
				}
				break;
			case 3:
				break;
			case 4:
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
					MEM_reset(0,EEPROM.length());
					LED_confirm();
					MEM_read();
					break;			
				}
				break;
			case 3: //All stop bit 5 van COM_reg

				switch (value) {
				case 0:
					COM_reg &= ~(1 << 4); ///com_reg bit 5 kun je niet gebruiken is voor wachten op DCC
					LED_off;
					break;
				case 1:
					COM_reg |= (1 << 4);
					PORTB |= (1 << 5);
					PORTB |= (1 << 4);
					PORTB |= (1 << 3);
						
					break;
				}

				break;
			case 4:
				if (EEPROM.read(501) != value) {
					EEPROM.write(501, value);
					LED_confirm();
					MEM_read();
				}
				break;
			case 5: //speed sunrise and sunset. value 1-255
				if ( EEPROM.read(510) != value) {
					EEPROM.write(510, value);
					LED_confirm();
					MEM_read();
				}
				break;
			case 6: //#502 weertype
				if (EEPROM.read(502) != value) {
					EEPROM.write(502, value);
					LED_confirm();
					MEM_read();
				}
				break;
			case 7: //#503 Model-tijd zonsopgang
				if (EEPROM.read(503) != value) {
					EEPROM.write(503, value);
					LED_confirm();
					MEM_read();
				}
				break;
			case 8://#504 Model-tijd Zonsondergang
				if (EEPROM.read(504) != value) {
					EEPROM.write(504, value);
					LED_confirm();
					MEM_read();
				}
				break;

			case 10: //instellen aantal leds NZ #500
				if (EEPROM.read(500) != value) {
					EEPROM.write(500, value);
					LED_confirm();
					MEM_read();
				}
				break;
				
				
			case 15: //enable lightning(true) #400 bit0
				
				if (value < 2) {					
					mr = EEPROM.read(400);
					mr = mr << 7;
					mr = mr >> 7;
					if (mr ^ value == 1) {
						if (value == 1) {
							COM_set |= (1 << 0);
						}
						else {
							COM_set &= ~(1 << 0);
						}
						EEPROM.write(400, COM_set);
						LED_confirm();
						MEM_read;
					}
				}
				break;

			}
		}
	}
}
void APP_VL(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	int VL_adresmin = (COM_DCCAdres * 4) + 1; //no mistake, COM_DCCadres for decoder = 1 lower, COM_DCCadres+1 1th led adres.
	int pixel;

	if (adres >= VL_adresmin & adres < VL_adresmin+32) {
		if (type == false) {//switch
			if (port == true) {
				led_vl[adres - VL_adresmin] = 0xAAAAAA; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
			}
			else {
				led_vl[adres - VL_adresmin] = 0x000000;
			}
			//COM_fastled();
			FastLED.show();
		}
		else {//CV
			pixel = adres - VL_adresmin;
			switch (cv) { //adres is altijd hier het adres van de decoder. Niet van het channel
				case 3: //switches on individual leds.
					//merk op volgorde= niet 11> rood 12 > groen 13> blauw maar 11> groen 12> rood 13> blauw
					switch (value) {
					case 11 :	
						led_vl[pixel].g = 0xFFFFFF;
						break;
					case 12:
						led_vl[pixel].r = 0xFFFFFF;
						break;
					case 13:
						led_vl[pixel].b = 0xFFFFFF;
						break;
					case 21:
						led_vl[pixel+1].g = 0xFFFFFF;
						break;
					case 22:
						led_vl[pixel+1].r = 0xFFFFFF;
						break;
					case 23:
						led_vl[pixel+1].b = 0xFFFFFF;
						break;
					case 31:
						led_vl[pixel + 2].g = 0xFFFFFF;
						break;
					case 32:
						led_vl[pixel + 2].r = 0xFFFFFF;
						break;
					case 33:
						led_vl[pixel + 2].b = 0xFFFFFF;
						break;
					case 41:
						led_vl[pixel + 3].g = 0xFFFFFF;
						break;
					case 42:
						led_vl[pixel + 3].r = 0xFFFFFF;
						break;
					case 43:
						led_vl[pixel + 3].b = 0xFFFFFF;
						break;
					}
					break;
								
				case 10: 					
					led_vlap[pixel] = value;
					if (EEPROM.read(pixel) !=value) EEPROM.write(pixel, value);
					break;
				case 11:
					pixel = pixel + 1;
					led_vlap[pixel] = value;
					if (EEPROM.read(pixel) != value) EEPROM.write(pixel, value);
					break;
				case 12:
					pixel = pixel + 2;
					led_vlap[pixel] = value;
					if (EEPROM.read(pixel) != value) EEPROM.write(pixel, value);
					break;
				case 13:
					pixel = pixel + 3;
					led_vlap[pixel] = value;
					if (EEPROM.read(pixel) != value) EEPROM.write(pixel, value);
					break;
			}

			//COM_fastled();
			FastLED.show();
		}
	}
}
void APP_FX(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	byte FX_adresmin = (COM_DCCAdres * 4) + 33; //VL has 32 adresses.
	if (adres >= FX_adresmin & adres <= FX_adresmin + 4) {

		//if (type == false) {//switch
		switch (channel) {
		case 1:
			if (port == true) {
				PRG_reg[6] |= (1 << 0);
			}
			else {
				PRG_reg[6] &= ~(1 << 0);
			}
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		}
	}
}
void LED_setPix(byte output, byte r,byte g,byte b) { 
	//sets new value for pixel RGB
	for (byte i = 0; i < 32; i++) { //check all leds in this group
			if (led_vlap[i] == output)led_vl[i] = CRGB(r, g, b);			
	}
	FastLED.show();

}
void LED_setLed(byte output, byte led, byte value) {
	//sets new value for 1 othe leds in a pixels
	if (led < 4) {
		for (byte i = 0; i < 32; i++) { //check all pixels
			if (led_vlap[i] == output) {
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
		}
		FastLED.show();
	}
	else {
		Serial.print(F(">>, "));
	}
}
void LED_program() {
	static byte s=0;
	LED_off();
		switch (s) {
			case 0:
				PORTB |= (1 << 5);
				s = 1;
				break;

			case 1:
				PORTB |= (1 << 4);
				s = 2;
				break;

			case 2:
				PORTB |=(1 << 3);
				s = 0;
				break;
		}
}
void LED_off() {
	//switches all indicator leds off
	PORTB &= ~(1 << 5);
	PORTB &= ~(1 << 4);
	PORTB &= ~(1 << 3);
	if (bitRead(COM_reg, 5) == false) {
		switch (bitRead(COM_reg, 2)) { /// 23mei2018********************************************************************************
		//switch(dagnacht){
		case true:
			PORTB |= (1 << 4);
			break;
		default:
			PORTB |= (1 << 5);
			break;
		}
	}
}
void LED_confirm() {
	//geeft een led signaal en delayed, blocked het hele systeem!!
	for (byte i = 0; i < 10; i++) {
		PORTB ^= (1 << 3);
		delay(50);
	}
	PORTB &= ~(1 << 3);
}
void PRG_dl(byte pn) {

	static byte fxb; //hoe ver het effect naar het westen
	static byte rled[3]; //willekeurige byte
	static byte dl_sp = 1; 	
	static byte maxclr[3]; //max te bereiken kleuren per weer type
	static byte ledcount = 0; // welke led
	//static byte rc = 0; //row count
	static byte dl_sc = 0; //laatste led, en teller in fase 113
	static byte teller = 0; //algemene teller	
	static byte dl_s = 0; //shift, aantal over geslagen leds dynamisch
	static byte dl_st = 0; //shift overgeslagen leds statisch
	static byte weer;
	static unsigned long dl_t = 0;	
	static byte fase = 0; 
			
	if (bitRead(PRG_reg[pn], 7) == true){ 	//if true day/night is manually changed by DCC or switch
		PRG_reg[pn] &= ~(1 << 7);
		teller = 0;
		ledcount = 0;
		fase = 0;
		PRG_reg[pn] &= ~(1 << 1); //disable modeltijd start

		if (bitRead(COM_reg, 2) == false) {
		//if(dagnacht==false){
			PORTB |= (1 << 5);
			PORTB &= ~(1 << 4);
			mt_hr = mt_zonop; //set modeltijd 
			mt_min = 0;
		}
		else {
			PORTB |= (1 << 4);
			PORTB &= ~(1 << 5);
			mt_hr = mt_zononder; //set modeltijd
			mt_min = 0;
		}
	}
	if (micros() - dl_t > (SrS*50)){ //snelheid in stellen met CV5 
		dl_t = micros();	
//*****************************switch fase
		switch (fase) {
		case 0:
			if (bitRead(COM_reg, 2) == false) { //sunrise
			//if(dagnacht==false){
				switch (CV_wt) {
					//0=random 1=sunny 2=clouded 3=miserable weather 4=no effects
				case 0:
					weer = random(1, 4); //0-1-2-3 (never 4)
					//weer = 3;
					break;
				default:
					weer = CV_wt;
					break;
				}

				Serial.print(F("Weer: "));
				Serial.println(weer);

				fxb = random(2, led_al * 7 / 10);
				switch (weer) {
				case 1: //zonnig
					dl_sp = 1;
					fase = 10;
					teller = 0;
					dl_st = random(1, 7); //aantal leds wat wordt overgeslagen. 
					//fxb = random(led_NZ, al * 7/10);
					break;

				case 2: //half bewolkt, vaak zon.
					//fxb = random(led_NZ, al * 7 / 10);
					fase = 20;
					break;

				case 3: //bewolkt weer geen kleureffecten
					fase = 40;
					if (bitRead(COM_reg, 4) == false) {
						maxclr[2] = random(170, 210);
						maxclr[0] = maxclr[2] - random(10, 50); maxclr[1] = maxclr[2] - random(0, 30);
					}
					else { //no effect
						maxclr[0] = 240; maxclr[1] = 240; maxclr[2] = 240;
					}
					teller = 0;
					break;

				case 4:
					fase = 50;
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
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					if (ledcount < fxb) led_dl[ledcount] = CRGB(5, 0, 0);
				}
			}
			if (ledcount == rled[1] & led_dl[ledcount].r != 2) led_dl[ledcount] = CRGB(3, 3, 3);
			if (ledcount >= led_al) {
				teller++;
				if (teller > led_al / 5) {
					teller = 0;
					fase = 13;
					maxclr[0] = random(50, 150); //te bereiken rode kleur instellen
					maxclr[1] = maxclr[0] * 4 / 10; //Geel waarde bijvoegen
				}
			}
			break;

		case 13:
			if (ledcount < fxb) {
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					wit(ledcount, 2, maxclr[0], 0, 0, false);
					if (led_dl[ledcount].r > maxclr[0])fase = 16;
				}
			}
			else {
				if (ledcount == rled[0])led_dl[ledcount] = CRGB(5, 5, 5);
			}
			break;
		case 16:
			if (ledcount - dl_sc > dl_st) {
				dl_sc = ledcount;
				if (ledcount < fxb & led_dl[ledcount].g < maxclr[1]) {
					led_dl[ledcount].g += 2;
					PRG_reg[pn] &= ~(1 << 6);
				}
			}
			if ((ledcount >= led_al) & (bitRead(PRG_reg[pn], 6) == true)) {
				fase = 19;
				teller = 0;
				maxclr[0] = 240; maxclr[1] = 240; maxclr[2] = 240;
			}
			if (ledcount == rled[0]) wit(rled[0], 6, maxclr[0], maxclr[1], maxclr[2], false);
			if (ledcount == rled[1])led_dl[ledcount] = CRGB(100, 40, 0);
			if (ledcount == rled[2] & led_dl[ledcount].r == 0)led_dl[ledcount] = CRGB(20, 20, 20);
			if (ledcount == rled[2] + 1 & led_dl[ledcount].r == 0)led_dl[ledcount] = CRGB(20, 20, 20);
			break;

			//stuk fine tuning weggehaald hier, diende om de uitleds 1 voor 1 aan te zetten, maar veroorzaakt een timing probleem
			//ik vermoed te snel achter elkaar een fastled.show


		case 19:
			//white out
			if (ledcount >= led_al) teller++;
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
			if ((ledcount >= led_al) & (bitRead(PRG_reg[pn], 6) == true)) fase = 1;
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
				//COM_fastled();
			}
			if (ledcount >= led_al) teller++;
			if (teller > (led_al * 7 / 10)) {
				teller = 0;
				fase = 41;
				maxclr[0] = random(180, 250);
				maxclr[1] = maxclr[0]; maxclr[2] = maxclr[0];
			}
			else {
				if (teller > (led_al * 1 / 10) & teller < (led_al * 2 / 10)) {
					if (ledcount == rled[1] & ledcount < fxb) led_dl[ledcount] = CRGB(250, 0, 0);
				}
				if (teller > (led_al * 2 / 10) & teller < (led_al * 4 / 10)) {
					if (ledcount == rled[2]) led_dl[ledcount] = CRGB(250, 130, 0);
				}
			}
			break;
			//**********WEER 4
		case 40: //weertype 4, slecht weer naar donker daglicht

			if (ledcount == rled[0] | ledcount == rled[1] | ledcount == rled[2]) {
				wit(ledcount, 6, maxclr[0], maxclr[1], maxclr[2], false);
				//COM_fastled();
			}
			if (ledcount >= led_al) teller++;
			if (teller > (led_al / 2)) { //var calc in fase 4
				fase = 41;
				teller = 0;
			}
			break;
		case 41: //to full daylight
			wit(ledcount, 1, maxclr[0], maxclr[1], maxclr[2], true);
			if (ledcount >= led_al & bitRead(PRG_reg[pn], 6) == true) fase = 1;
			break;

		case 50: //no effect direct full scale white
			led_dl[ledcount] = 0xFFFFFF;
			if (ledcount >= led_al)fase = 1;
			break;

			//******************SUNSET******SUNSET********SUNSET*******
		case 100: //sunset	
			fase = 110;
			break;

		case 110:
			//if (ledcount>=led_al) teller++;
			//if (teller < 150) {
			zwart(ledcount, 2, 70, 70, 70, true);
			if (ledcount == rled[0])led_dl[ledcount] = 0x000000;
			//}
			//else {
			if (ledcount >= led_al & bitRead(PRG_reg[2], 6) == true) {
				fase = 112;
				maxclr[1] = maxclr[0] * 3 / 10;
				//teller = 0;
			}
			break;
			

		case 112:
			if (ledcount > fxb) {
				if (ledcount - dl_sc > dl_st) {
					dl_sc = ledcount;
					zwart(ledcount, 1, 200, maxclr[1], 0, true);
				}
				else {
					zwart(ledcount, 1, 15, 15, 15, false);
					if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
				}
			}
			else {
				zwart(ledcount, 1, 20, 20, 20, false);
				if (ledcount == rled[1])led_dl[ledcount] = CRGB(0, 0, 0);
			}
			if ((ledcount >= led_al) & (bitRead(PRG_reg[pn], 6) == true))fase = 114;
			break;

		case 114:
			if (ledcount >= led_al) {
				teller++;
			}

			if (teller < 40) {
				zwart(ledcount, 1, 16, 16, 15, false);
				if (ledcount == rled[0])led_dl[ledcount] = CRGB(55, 20, 0);
				if (ledcount == rled[2])led_dl[ledcount] = 0x000000;
				if (ledcount == rled[1])led_dl[ledcount] = 0x000000;
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
					zwart(ledcount, 1, 200, 0, 0, true);
				}
			}
			else
			{
				if (ledcount == rled[0])led_dl[ledcount] = 0x000000;
				if (ledcount == rled[1])led_dl[ledcount] = 0x000000;
				if (ledcount == rled[2])led_dl[ledcount] = 0x000000;
			}
			if ((ledcount >= led_al) & (bitRead(PRG_reg[pn], 6) == true))fase = 200; //stop
			if (ledcount == rled[0]) led_dl[ledcount] = 0x000000;

			break;

		case 120:	 //sunset half bewolkt			
			maxclr[0] = random(40, 90); maxclr[1] = maxclr[0]; maxclr[2] = maxclr[0];
			fase = 122;
			break;

		case 122:
			zwart(ledcount, 2, maxclr[0], maxclr[1], maxclr[2], true);
			if ((ledcount >= led_al) & (bitRead(PRG_reg[pn], 6) == true))fase = 123;
			if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
			break;

		case 123:
			if (ledcount >= led_al) teller++;
			if (teller < (led_al * 4/ 10)) {
				zwart(ledcount, 1, 15, 15, 15, false);
				if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
				if (ledcount == rled[2] & ledcount > fxb)led_dl[ledcount] = CRGB(90, 30, 0);
				if (ledcount == rled[1])led_dl[ledcount] = CRGB(100, 30, 0);
			}
			else {
				fase = 200;
			}
			break;

		case 140: //begin sunset bewolkt weer, geen effecten
			fase = 141;
			break;

		case 141:
			zwart(ledcount, 1, 30,30 , 30, true);
			if (ledcount == rled[0])led_dl[ledcount] = CRGB(0, 0, 0);
			if ((ledcount >= led_al) & (bitRead(PRG_reg[pn], 6) == true)) {
				fase = 200;
			}
			break;

		case 150: //no effect direct black
			led_dl[ledcount] = 0x000000;
			if (ledcount >= led_al)fase = 1;
			break;

		case 200: //black out naar nacht
			fase = 201;
			teller = 0;
			break;

		case 201: //reduce pixels to 20% of total pixels
			//first count active pixels
			zwart(ledcount, 1, 10, 10, 10, false);
			if (led_dl[ledcount]) {
				teller++;
			}
			if (ledcount >= led_al) {
				if (teller > (led_al /4)) {
					fase = 202;
					teller = 0;	
				}
				else {
					fase = 203;

				}			
			}
			break;

		case 202:
			if (rled[0] == ledcount | rled[1]==ledcount | rled[2]==ledcount ) {

				for (byte i = 0; i < 10; i++) {
					if (led_dl[ledcount + i]) {
						led_dl[ledcount + i] = 0x000000;
						i = 20;
					}

					for (byte i = 0; i <10; i++) {
						zwart(ledcount + i, 1, 1, 1, 1, false);
					}
				}
			}
			if (ledcount >= led_al)fase = 201;
			break;

		case 203:		
			zwart(ledcount, 1, 1, 1, 1, true);
			if (ledcount >= led_al & bitRead(PRG_reg[2], 6) == true)fase = 204;
			break;	
		case 204:
			dl_sp = (led_dl[ledcount].r + led_dl[ledcount].g + led_dl[ledcount].b);
			if (dl_sp == 1 | dl_sp == 2) led_dl[ledcount] = 0x010101;
			if (ledcount >= led_al) fase = 205;
			break;

		case 205:
			if (ledcount == rled[0]) {
				for (int i = 0; i < led_al; i++) {
					dl_sp = ledcount + i;
					if (dl_sp > led_al) dl_sp = 0;
					if (led_dl[dl_sp]) {
						led_dl[dl_sp] = 0x000000;
						i = led_al;						
					}
				}
			}
			if (ledcount >= led_al) fase = 206;			
			break;
						
			case 206:
				if (led_dl[ledcount])teller++;
			
				if (ledcount >= led_al) {
					
					if (teller > 8)	{ // (led_al / 15)) {
						fase = 205;	
						teller = 0;	
					}
					else {
						fase = 210;
					}
				}
				break;

			case 210:
				if (led_dl[ledcount]) { 
					if (led_dl[ledcount + 1])led_dl[ledcount+1] = 0x000000;
				}
				if (ledcount >= led_al)fase = 1;
				break;			
}
//**********************end switch fase		
			ledcount++;
			if (ledcount > led_al){//merk op 1 verder als in de switch case
				//COM_fastled();
				FastLED.show();
				ledcount = 0;
				PRG_reg[pn]|= (1 << 6); //flag voor bereiken eindwaarde				
				rled[0] = random(0, led_al); 
				rled[1] = random(0, led_al);
				rled[2] = random(0, led_al); 
				randomSeed(analogRead(A5));	//port (pinA5) must be free, not in use
				
				dl_sc = 0;
				dl_s++;
				if (dl_s > 5) dl_s = 0;

				if (bitRead(COM_reg, 2) == false) { //flashes the led status
					//if (dagnacht==false){
					PORTB ^= (1 << 5);
				}
				else {
					PORTB ^= (1 << 4);
				}
			}	

			//STOP PROGRAM (bit7 of register = true)
			if (bitRead(PRG_reg[pn],7)==true) { //stop this program plus init all variables
				Serial.println(F("Stop prg"));
				PRG_reg[pn] &= ~(1 << 7);
				ledcount = 0;
				dl_sc = 0;
				dl_s = 0;
				fase = 0;	
				PRG_reg[pn] &= ~(1 << 0); //disable active

				PRG_reg[pn] |= (1 << 1); //enable model time start

				if (bitRead(COM_reg, 2) == false) { //stops flashing led
				//if(dagnacht==false){
					PORTB |= (1 << 5);

					PRG_hr[2] = mt_zononder;
					PRG_min[2] = 0;					
				}
				else {
					PORTB |= (1 << 4);
					PRG_hr[2] = mt_zonop;
					PRG_min[2] = 0;
				}
			}
	}
}
void PRG_lightning() { //Programnummer=4, lighting starts now
	byte lgt_count = led_al * 5 / 100; //% van leds als lightning
	static byte lgt_led;
	static byte lgtfase;
	static unsigned long time; 
	static unsigned int interval;
	static byte atl; //aantal flitsen achter elkaar 
	static byte afl; //aantal leds in de bliksemflits
	static byte br; //helderheid
	static byte minute;
	static byte mc=0;
	static byte mcc = 0;
	static unsigned int duur; //hoelang moet het onweren
	static unsigned int it=0; //intensiteit van onweer

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
	
	if (bitRead(PRG_reg[4], 3) == false) {  ///hier ziet het fout volgens mij, stond een 1 3 van gemaakt, weer 1 van gemaakt op 3-6
		PRG_reg[4] &= ~((1 << 1)); //disable modeltijd start    DIT toegevoegd 23/5 maar een keer starten vanuit modeltijd

		//Serial.println(F("flash"));

		PRG_reg[4] |= (1 << 3);
		PRG_reg[4] |= (1 << 0);//active prg 4
			
		lgtfase = 0;
		time=millis();
		interval = 0;
		mc = 0;
		duur =  random(60, 300); //hoe lang het moet onweren
		//Serial.print(F("Duur: "));
		//Serial.println(duur);
		it = random(6000, 12000);
	}
	else { //prg 4 =active bit 1
		if (millis() - interval > time) { 
			switch (lgtfase) {
			case 0: //start
				afl =  random(2, lgt_count);
				lgt_led = random(0, led_al - afl);//bepaal startled in hemel	
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
				FastLED.show(); //8juni2018
				break;

			case 20:
				for (int i = 0; i < afl; i++) {
					led_dl[lgt_led + i]=led_lgt[i]; //waarde led herstellen			
				}
				atl--;
				if (atl > 0) {
					interval = random(10, 50);
					lgtfase = 10;
					}
					else {
						if (mc > duur) {
							//stop prg_lightning
							PRG_reg[4] &= ~((1 << 0));
							PRG_reg[4] &= ~((1 << 3));
							PRG_reg[4] &= ~((1 << 1)); //disable modeltijd start
							//Serial.println(F("Flash end"));
						}
						else {
							lgtfase = 0;
						}
					}
					FastLED.show();
				break;
			}
		time = millis();
		}
	}
}
void dld_exe() { 
	for (byte i = 0; i < led_al; i++) {
		if(bitRead(COM_reg,2)==false){
		//if(dagnacht==false){
			led_dl[i] = CRGB(250, 250, 250);
		}
		else {//nacht	
			led_dl[i] = CRGB(0, 0, 0);
		}
	}
		mt_hr = mt_zonop;
		if(bitRead(COM_reg,2)==true) mt_hr = mt_zononder;
		//if (dagnacht == true) mt_hr = mt_zononder;
		mt_min = 0;	
		BLD_reset();
		FastLED.show();
}
void dld_com(byte st) { //=dag/nacht 0=toggle 1=day 2=night
		switch (st) {
		case 0:
			COM_reg ^= (1 << 2); //bit3 false = day, true is night	
			/*
			if (dagnacht == false) {
				dagnacht = true;
			}
			else {
				dagnacht = false;
			}
			*/

			break;
		case 1:
			COM_reg &= ~(1 << 2);
			//dagnacht = false;
			break;
		case 2:
			COM_reg |= (1 << 2);
			//dagnacht = true;
			break;
		}
		//PRG_reg[3] |= (1 << 0);//start program 3
		dld_exe();
		PRG_reg[2] &= ~(1 << 0); //Stop program 2, 
		PRG_reg[2] &= ~(1 << 1); //disable modeltime start				

		if (bitRead(COM_reg, 2) == false) {
		//if(dagnacht==false){
			PORTB |= (1 << 5);
			PORTB &= ~(1 << 4);
			PRG_reg[4] &= ~(1 << 0);
			PRG_reg[4] &= ~(1 << 3);
			PRG_reg[4] &= ~(1 << 1);
		}
		else {
			PORTB |= (1 << 4);
			PORTB &= ~(1 << 5);
		}
}
void PRG_traffic(int pn) {
//verkeerslicht, starts and runs forever...
	
	static unsigned long tijd;
	static unsigned int periode=500;
	static byte fase=0;
	static byte WieMag;


	//dit is niet goed meer.... 

	static byte out;
	static byte output1=20;
	static byte output2=21;
	
	

	if (bitRead(PRG_reg[pn], 3) == false) { 
		PRG_reg[pn] |= (1 << 3); //no init 
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
				out = output2;
				periode = 5000;
				LED_setPix(output1, 200,0,0);
				LED_setPix(output2, 200,0,0);
				break;
			case 1:
				periode = 100;
				fase = 2;

				switch (out) {
				case 4:					
					out = output2;
					break;
				case 5:
					out = output1;
					break;
				}

			case 2:
				periode = 12000;
				LED_setPix(out, 0,200,0);
				fase = 3;
				break;

			case 3: //OW oranje
				periode = 4000;
				LED_setPix(out, 0,0,150);
				fase = 4;
				break;
			case 4:
				//all red
				periode = 5000;
				LED_setPix(output1, 200,0,0);
				LED_setPix(output2, 200,0,0);
				fase = 1;
				break;
			}
			//COM_fastled();
		}
	}
}
void PRG_las() { //programma 6
	//simutatie lassen
	 byte out = 6;
	static unsigned long t;
	if (millis() - t > 20) {
		t = millis();
		PRG_reg[6] ^= (1 << 7);
		if (bitRead(PRG_reg[6], 7) == true) {
			LED_setPix(out, 0, 0, 0);
		}
		else {
			LED_setPix(out, 255, 255, 255);
		}
	}
}
void PRG_huis(byte pg, byte out,byte huis) { //pg=program out=output huis=building pixel
	
	PRG_reg[pg] = PRG_reg[pg] >> 2; 
	byte hk; //huiskamer
	byte sk; //slaapkamer
	byte wc; //badkamer, wc
	byte bl; //buitenlicht
	byte hl; //hal

	switch (huis) { //huis programmaas
	case 1: //h1-h2-h4
		hk = 1;
		sk = 0;
		wc = 2;
		bl = 10;
		hl = 10;
		break;
	case 2: //h3
		hk = 1;
		sk = 2;
		wc = 0;
		bl = 10;		
		hl = 10;
		break;
	case 3: //h5
		hk = 1;
		sk = 0;
		wc = 10;
		bl = 10;
		hl = 2;
		break;
	case 4: //h6
		hk = 1;
		sk = 0;
		wc = 10;
		bl = 2;
		hl = 10;
		break;
	case 5:
		hk = 2;
		sk = 10;
		wc = 10;
		bl = 0;
		hl = 1;
		break;
	}
	PRG_reg[pg] = PRG_reg[pg] + 1;

	switch (PRG_reg[pg]){ //max 63 stappen...
	case 1:

		LED_setLed(out, bl, 250);//buitenlicht aan
		interval(pg, random(5, 15), 0);
		break;

		
	case 2:		
		LED_setLed(out, hk, 250);//huiskamer aan
		interval(pg,random(10,60), 0);
		break;
	case 3:
		LED_setLed(out, hl, 250);//hal aan
		interval(pg, random(2, 5), 0);
		break;
	case 4:
		LED_setLed(out, wc, 250);//WC aan
		interval(pg, random(10,20), 0);
		break;
	case 5:
		LED_setLed(out, wc, 0);// WC uit
		interval(pg, random(2,5), 0);
		break;
	case 6:
		LED_setLed(out, hl, 0);//hal uit
		interval(pg, random(10,60), 0);
		break;
	case 7:
		LED_setLed(out, hl, 250);//hal aan 
		interval(pg, random(2, 4), 0);
		break;
	case 8:
		LED_setLed(out,wc, 250);//wc aan
		interval(pg, random(10,20), 0);
		break;
	case 9:
		LED_setLed(out, wc, 0);//WC uit
		interval(pg, random(2,6), 0);
		break;	
	case 10:
		LED_setLed(out, hl, 0);//hal uit
		interval(pg, random(2, 10), 3);
		break;
	case 11:
		LED_setLed(out, sk, 250);//slaapkamer aan
		interval(pg, random(5,20), 0);
		break;
	case 12:
		LED_setLed(out, wc, 250); //WC aan
		interval(pg, random(2,15), 0);
		break;
	case 13:
		LED_setLed(out, hl, 3);//hal zwak
		interval(pg, random(2, 5), 0);
		break;
	case 14:
		LED_setLed(out, hk, random(0,3)); //huiskamer zwak of uit
		interval(pg, random(2,15), 0);
		break;
	case 15:
		LED_setLed(out, wc, 0); //WC uit
		interval(pg, random(5,20), 0);
		break;
	case 16:
		LED_setLed(out, sk, 2); //slaapkamer zwak
		interval(pg, random(10,25), 0);
		break;
	case 17:
		LED_setLed(out, sk, 0); //slaapkamer uit
		interval(pg, random(60, 180), 0);
		break;
	case 18:
		LED_setLed(out, sk, 3); //slaapkamer zwak
		interval(pg, random(2, 15), 0); 
		break;
	case 19:
		LED_setLed(out, wc, 250); //WC aan
		interval(pg, random(10, 30), 0); 
		break;
	case 20:
		LED_setLed(out, wc, 0); //WC uit
		interval(pg, random(2, 15), 0); //wake up sunrise
		break;
	case 21:
		LED_setLed(out, sk, 0); //slaapkamer uit
		interval(pg, random(20, 60), 1); //wake up sunrise
		break;
	case 22:
		LED_setLed(out, sk, 250); // slaapkamer aan
		interval(pg, random(5,15), 0);
		break;
	case 23:
		LED_setLed(out, wc, 255);//wc aan
		interval(pg, random(10,20), 0);
		break;
	case 24:
		LED_setLed(out, wc, 0);//wc uit
		interval(pg, random(5,12), 0);
		break;
	case 25:
		LED_setLed(out, sk, 0); //slaapkamer uit
		interval(pg, random(5,15), 0);
		break;
	case 26:
		LED_setLed(out, hk, 255);//huiskamer aan
		interval(pg, random(15,60), 0);
		break;
	case 27:
		LED_setLed(out, hk, 0);//huiskamer uit
		interval(pg,random(5,15), 0);
		break;
	case 28:
		LED_setLed(out, hl, 0);//hal uit
		interval(pg, random(3, 10), 0);
		break;
	case 29:
		LED_setLed(out, bl, 0);//buitenlicht uit
		interval(pg, 10, 0);
		break;	

	
	case 30: //last step
			 //reset startwaardes
		
		PRG_hr[pg] = mt_zononder;
		PRG_min[pg]= random(20, 60);	


		//***		
		Serial.print(F("stp prgr:  "));
		Serial.print(pg);
		Serial.print(",  ");
		Serial.print(F("#:  "));
		Serial.println(PRG_reg[pg]);
		//***
		PRG_reg[pg] = 0;
		break;
		
	}
	//build program register
	PRG_reg[pg]= PRG_reg[pg] << 2;
	PRG_reg[pg] |= (1 << 1);
	
}
void BLD_reset() {
//resets all building vl pixel parameters, after power-up and manual switching between day and night.
	Serial.println(F("BLD_reset"));

			//alle pixels in VL uitzetten
			for (byte i = 0; i < 32; i++) { //BELANGRIJK AANTAL PIXELS GELIJK AAN DECLARATIE
				led_vl[i] = 0x000000;
			}
			FastLED.show();
			//COM_reg |= (1 << 1);

	for (byte i = 10; i < 20; i++) { //3jun2018 10-19 huisprogrammaas 
		//programs building start at sunset			
			PRG_hr[i] = mt_zononder;
			PRG_min[i] = random(0, 59); //mag langer naar 60 ofzo
			PRG_reg[i] = 2;
	}
}
void DSP_shift() {
/*
functie zet de twee bytes in de beide schuifregisters eerst shft[0] (segment) naar shift 2 daarna shft[1] (digits) naar shift 1
pin 4 portd4 = data
pin 5 portd5= shift clock SRCLK
pin 6 portd6= latch clock RCLK

 */
	// gebruik general porpose register GPIOR0 = 0;


	static byte fase = 0;
	static byte bitcount = 0;

	switch (fase) {
	case 0: //start new cycle
		//GPIOR0 &= ~(1 << 0); //clear bit 0
		bitcount = 0;
		fase = 10;
		break;
	case 10:
		if (bitRead(shft[bitRead(GPIOR0, 0)], bitcount) == true){
			PORTD |= (1 << 4);  
		}
		else {
			PORTD &= ~(1 << 4);
		}
		bitcount++;

		if (bitcount > 7) {
			bitcount = 0;
			GPIOR0 ^= (1 << 0); //toggle bit 0

			if (bitRead(GPIOR0, 0) == false) {
				fase = 40;  //make latch puls
			}
			else { //next byte
				fase = 10;
			}
		}
		else {
			fase = 20;
		}
		break;

	case 20: 
		//shift puls maken,eerst doen op volle snelheid, eventueel kan hier een teller timer tussen komen
		PORTD |= (1 << 5);
		fase = 30;
			break;
	case 30:
		PORTD &= ~(1 << 5);
		fase = 10; //next bit
		break;
	case 40:
		//latch puls maken, eerst volle snelheid
		PORTD |= (1 << 6);
		fase = 50;
		break;
	case 50:
		PORTD &= ~(1 << 6);
		fase = 0;
		GPIOR0 &= ~(1 << 1); //disable DSP_shift()
		break;
	}
}
void interval(byte prg,byte tijd,byte type) {

	//berekend de nieuwe starttijd voor een programma.
	//type 0=huidige tijd plus interval 1=zonsopgang plus interval 2=zonsondergang plus interval 3=24hr plus interval
	//check en correct lengte interval meer dan 60minuten.
	byte hr=0;	
	PRG_min[prg] = mt_min + tijd;	
	while (PRG_min[prg] > 59) {
		PRG_min[prg] = PRG_min[prg] - 60;
	hr++;
	}	
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

	Serial.print(F("Prg: "));
	Serial.print(prg);
	Serial.print(",  ");
	Serial.print(F("uur:  "));
	Serial.print(PRG_hr[prg]);	
	Serial.print(",  ");
	Serial.print(F("minuut:  "));
	Serial.println(PRG_min[prg]);

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
void zwart(byte led, byte dec, byte mr, byte mg, byte mb, boolean stop) {
	 byte temp;
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
}
void lightningstart(byte kans) { //start lightningeffect op tijd
//kans....//	0=nooit, 1=altijd, 2=50%, 3=33%,  4=25%	
	 byte temp;	
	if (bitRead(COM_set, 0) == true) { //enable lighting		
		temp = random(1, kans + 1);
		if (temp == 1) {
			//temp = 0;
			//Serial.println(F("onweer"));
			//while (temp != mt_zonop) {
			//	temp++;
			//} //hoeveel uur tot zonsopgang
			temp = 24 - (mt_zononder - mt_zonop);
			//set wakeuptime for lightning
			PRG_hr[4] = mt_zononder+random(0, temp);
			if (PRG_hr[4] > 23)PRG_hr[4] = PRG_hr[4] - 24;
			PRG_min[4] = random(0, 59);
			PRG_reg[4] |= (1 << 1); //enable modeltijd start
		}
	}
}
void loop() {

	/*
	
	static byte dl;
//	if (dl != bitRead(COM_reg, 2)) {
	if (dl!=dagnacht){
		dl = dagnacht; // bitRead(COM_reg, 2);
		Serial.print("dagnacht is changed...");
		Serial.println(dagnacht);
	}
*/

	DEK_DCCh();
COM_Clk();
COM_ProgramAssign();
COM_switch();
if (bitRead(GPIOR0, 1) == true) DSP_shift();
}
