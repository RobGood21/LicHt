/*
 Name:		LicHt.ino
 Created:	12/23/2017 10:57:27 AM
 Author:	Rob Antonisse
 Version: 1.01

 LicHT is a project for full automated lighting of the model railroad, including day and night cyclus.
 Based on WS2811 adressable led control. 

*/

#include <FastLED.h>
#include <eeprom.h>


#define LPdl 8 //pin waar de daglicht ledstring op komt
#define LPvl 7 //pin voor verlichting string


byte COM_DCCAdres= 64;// default waarde, programmable only by button 2

//programmable by CV number
//#500 CV2, value 10 reset EEPROM
byte led_al=240; //#500 CV10, aantal leds max 240
byte tday = 20; //#501 CV4, tday how long is a modeltimeday in minute 24 is good value lager dan 10 werkt het geheel niet goed
byte CV_wt = 0;//#502 CV6, Weertype
byte mt_zonop=7; //503
byte mt_zononder = 21; //504 
byte SrS = 30; //#510 CV5, sun rise speed in micros/40 

CRGB led_dl[240]; //max aantal pixels
CRGB led_vl[32];
//tbv van prg_lightning
CRGB led_lgt[12]; //max aantal leds voor bliksem, dit dient  als geheugen voor de 'oude'waarde van de led
byte lgt_count; //aantal leds gebruikt voor bliksem
byte led_vlap[32]; //vlap=verlichting assign program, assign a program to a led
//byte led_evap[16];//evap=event assign program

byte BLD_clr[3]; //color of pixel in progress
byte COM_reg; //bit0 test PRG active(true)
//bit1 free
//bit3 day or night (also manual) false=day, true = night
//bit4 sunset and sunrise without effects true, false = with effects
//bit5 All stop, disable all prgrams for during programming true is disabled false=enabled programming mode
byte COM_set=0xFF; //default value
//bit0=Lighting on(true) or off(false)
byte PRG_reg[32]; //prg_reg 
//bit0=init 
//bit1=active 
//bit2=Start by model time
//bit3=free
//bit4=free
//bit5=free
//bit 6 flag voor bereiken eindwaarde
//bit7=flag end program

byte SW_reg; //register booleans for the switch states
/*
bit0 = status switch A0  (dag nacht)
bit1=status switch A1 (clock/ program)
*/

byte PRG_min[32]; //Time next active minute
byte PRG_hr[32]; //Time next actice hour
unsigned long Clk;
unsigned int mt; //modeltime minute
byte mt_min=1; //modeltimeclock minutes 
byte mt_hr=0; //modeltimeclock hours


//basic adres, adres Daylight decoder DL +1 
//VL (verlichting) decoder 16 adresses higher
//EV (events) decoder 16adresses higher

int FX_adresmin;
int VL_adresmin;

volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
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
	MEM_read();
	
	DDRB |= (1 << 5);	//pin13
	DDRB |= (1 << 4);  //Pin12 als output
	DDRB |= (1 << 3); //PIN11 as output

	DDRC &= ~(1 << 0); //set PINA0 as input
	DDRC &= ~(1 << 1); //set PINA1 as input

	Clk = millis();	
	//FastLED.setMaxPowerInVoltsAndMilliamps(5, 7000);	
	
	//instellen modeltijd bij power up
	mt_hr = mt_zonop;
	mt_min = 1;

	randomSeed(analogRead(0));

	//FastLED.setMaxPowerInVoltsAndMilliamps(5, 8000);	
	//FastLED.setBrightness (255); //beter niet

	//DeKoder part, interrupt on PIN2
	DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1	
}
void MEM_read() {
	//merk op volgorde van variable instellen is belangrijk, eerst COM_DCCadres en COM_DCCadres instellingen
	if (EEPROM.read(0) != 0xFF) COM_DCCAdres = EEPROM.read(0);
	if (EEPROM.read(400) != 0xFF) COM_set = EEPROM.read(400); //#400 COM_set register
	
	if (EEPROM.read(500) != 0xFF) led_al = EEPROM.read(500); //aantal pixels in Daglicht
	FastLED.addLeds<NEOPIXEL, LPdl>(led_dl, led_al);//create strip of 32leds on pin7 'verlichting' Xx leds on pin 8 'Daglicht'
	FastLED.addLeds<NEOPIXEL, LPvl>(led_vl, 32);//create strip of 32leds on pin7 'verlichting'
	
	if (EEPROM.read(501) != 0xFF) tday = EEPROM.read(501); //duur van model-tijd dag instellen
	mt = (tday * 1000) / 24;	

	if (EEPROM.read(502) != 0xFF) CV_wt = EEPROM.read(502); //Weertype 0=random 1=zon 2=halfbewolkt 3=bewolkt
	if (EEPROM.read(503) != 0xFF) mt_zonop = EEPROM.read(503); //Model-tijd zonsopgang tijd
	if (EEPROM.read(504) != 0xFF) mt_zononder = EEPROM.read(504); //Model-tijd Zonsondergang tijd
	
	if (EEPROM.read(510) != 0xFF)SrS = EEPROM.read(510); //speed of sunrise
	lgt_count = led_al * 5 / 100; //% van leds als lightning

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

	for (int i = 0; i < 32; i++) {
		led_vlap[i] = i + 1;
	}


	//geheugen toewijzing pixel aan programma nummer nu max 32 pixels
	for (int i = 1; i < 33; i++) {
		if (EEPROM.read(i) < 0xFF) led_vlap[i] = EEPROM.read(i);
	}
VL_adresmin = (COM_DCCAdres * 4) + 1; //no mistake, COM_DCCadres for decoder = 1 lower, COM_DCCadres+1 1th led adres.
FX_adresmin= (COM_DCCAdres * 4) + 33; //VL has 32 adresses.

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
	if (n > 12)n = 0;
}
void COM_fastled() {
	
	//refreshes the pixels in fastled
	COM_reg |= (1 << 1);
	if (DEK_Status < 2) {
		FastLED.show(); //only when there is no DCC command being processed 
		COM_reg &=~(1 << 1);
	}		
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
		if (bitRead(PRG_reg[2], 2) == true) {
			PINB |= (1 << 3); // toggle led on pin 11, toont dag nacht automatisch
		}
		else {
			if (bitRead(PORTB,3)==true) PORTB &= ~(1 << 3);
		}
		
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
			case 6:
				//temp always on
				//PRG_reg[6] |= (1 << 0);
				//no init

				break;
			case 10:
				//program always active, no init needed
				PRG_reg[10] |= (1 << 0);
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

	if (bitRead(COM_reg, 5) == false & bitRead(COM_reg,4) ==false) {	//enable normal mode
		switch (pn) {
		case 2: //schakel daglicht in of uit
			PRG_dl(pn);
			break;
		case 3:
			PRG_dld(pn);
			break;
		case 4:
			PRG_lightning();
			break;
		case 6:
			//PRG_las();
			break;
		case 10:
			//PRG_build();
			break;
		case 11:
			break;
		case 12:
			PRG_traffic(pn);
			break;

		default:
			//all not defined programs// do nothing
			break;
		}
	}
}
void COM_switch() {
	//handles the manual switches in the project
	static long Sw_time = 0;
	static unsigned int count = 0;
	static unsigned int pc = 0; //program count
	static boolean dld = 0;
	if (millis() - Sw_time >50) { //every xxms
		if (bitRead(COM_reg, 1) == true)COM_fastled(); //start fastled if 

		Sw_time = millis(); //reset counter
		//prgram switch op A1

		//tbv leds blinking in programmode, waiting for DCC adres
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
					//pc = 0;
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

				if (count > 40 & dld==false ) { //button hold > 2 seconds
					//Serial.println("lang ingedrukt");
					dld = true;
					dndirect(0);
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
	static byte mr; //=memorie read
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
					COM_reg &= ~(1 << 3);
				}
				else {
					COM_reg |= (1 << 3);
				}

				PRG_reg[2] |= (1 << 0); //start PRG_dl
				PRG_reg[2] |= (1 << 7); //Set changed flag

				break;

			case 2:
				if (bitRead(PRG_reg[3], 0) == false) { //only when program is in-active
					if (port == true) {
						dndirect(1);
						Serial.println("aan");
					}
					else {
						Serial.println("uit");
						dndirect(2);
					}
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
//Serial.println(decoder);
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
	//Verlichting 32leds

	//******AANTALLEN en adressen aanpassen straks....
	int pixel;
	if (adres >= VL_adresmin & adres < VL_adresmin+32) {
		if (type == false) {//switch
			if (port == true) {
				led_vl[adres - VL_adresmin] = 0xAAAAAA; //adres(DCC) minus adresmin geeft hier het led nummer in de rij.
			}
			else {
				led_vl[adres - VL_adresmin] = 0x000000;
			}
			COM_fastled();
		}
		else {//CV
			pixel = adres - VL_adresmin;
			switch (cv) { //adres is altijd hier het adres van de decoder. Niet van het channel
				case 3: //switches on individual leds.
					switch (value) {
					case 11 :	
						led_vl[pixel].r = 0xFFFFFF;
						break;
					case 12:
						led_vl[pixel].g = 0xFFFFFF;
						break;
					case 13:
						led_vl[pixel].b = 0xFFFFFF;
						break;
					case 21:
						led_vl[pixel+1].r = 0xFFFFFF;
						break;
					case 22:
						led_vl[pixel+1].g = 0xFFFFFF;
						break;
					case 23:
						led_vl[pixel+1].b = 0xFFFFFF;
						break;
					case 31:
						led_vl[pixel + 2].r = 0xFFFFFF;
						break;
					case 32:
						led_vl[pixel + 2].g = 0xFFFFFF;
						break;
					case 33:
						led_vl[pixel + 2].b = 0xFFFFFF;
						break;
					case 41:
						led_vl[pixel + 3].r = 0xFFFFFF;
						break;
					case 42:
						led_vl[pixel + 3].g = 0xFFFFFF;
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
			COM_fastled();
		}
	}
}
void APP_FX(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
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
void LED_set(byte output, byte r,byte g,byte b) { 
	for (byte i = 0; i < 32; i++) { //check all leds in this group
			if (led_vlap[i] == output)led_vl[i] = CRGB(r, g, b);			
	}
	COM_fastled();
}
void LED_go(byte prg) {
	for (byte i = 0; i < 32; i++) { //check all leds in this group
		if (led_vlap[i] == prg)led_vl[i] = CRGB(BLD_clr[0], BLD_clr[1],BLD_clr[2] );
	}
	COM_fastled();
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
		switch (COM_reg, 3) {
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
	//prg_reg 
	//bit0=init 
	//bit1=active 
	//bit2=Start by model time
	//bit3=free
	//bit4=free
	//bit5=
	//bit 6 flag voor bereiken eindwaarde
	//bit7=flag end program
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
			
	if (bitRead(PRG_reg[pn], 7) == true){ 	//if true day/night is changed (com_reg bit 3)
		PRG_reg[pn] &= ~(1 << 7);
		teller = 0;
		ledcount = 0;
		fase = 0;
		PRG_reg[pn] &= ~(1 << 2); //disable modeltijd start

		if (bitRead(COM_reg, 3) == false) {
			PORTB |= (1 << 5);
			PORTB &= ~(1 << 4);
			mt_hr = mt_zonop; //set modeltijd 
		}
		else {
			PORTB |= (1 << 4);
			PORTB &= ~(1 << 5);
			mt_hr = mt_zononder; //set modeltijd
		}
	}
	if (micros() - dl_t > (SrS*50)){ //snelheid in stellen met CV5 
		dl_t = micros();	
//*****************************switch fase
		switch (fase) {
		case 0:
			if (bitRead(COM_reg, 3) == false) { //sunrise
				switch (CV_wt) {
					//0=random 1=sunny 2=clouded 3=miserable weather 4=no effects
				case 0:
					//weer = random(1, 4); //0-1-2-3 (never 4)
					weer = 1;
					break;
				default:
					weer = CV_wt;
					break;
				}

				Serial.print("Weertype: ");
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
						//Serial.print("rood= ");
						//Serial.print(maxclr[0]);
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
				COM_fastled();
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
				COM_fastled();
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
					fase = 206;
					Serial.println("206");
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

		case 206:		
			//PRG_reg[2] ^= (1 << 5);

			//if (bitRead(PRG_reg[2], 5) == true) {			//??????
			if (led_dl[ledcount].r > 1) led_dl[ledcount].r--; // = led_dl[ledcount].r - (1 + led_dl[ledcount].r / 10);
			if (led_dl[ledcount].g > 1)led_dl[ledcount].g--;// = led_dl[ledcount].g - (1 + led_dl[ledcount].g / 10);
			if (led_dl[ledcount].b > 1)led_dl[ledcount].b--;// = led_dl[ledcount].b - (1 + led_dl[ledcount].b / 10);
			//}

			if (rled[0] == ledcount | rled[1] == ledcount) {

				for (byte i = 0; i < 10; i++) { //reduces series of remaining on pixels
					if (led_dl[ledcount + i]) {
						if (led_dl[ledcount + (i + 1)]) {
							led_dl[ledcount + (i + 1)] = 0x000000;
						}
						else {led_dl[ledcount + i] = 0x000000;
						}											
						i = 20;
					}
				}
			}
			if (ledcount >= led_al) {
				fase = 207;
				Serial.println("207");
			}
				

			//dl_sp++;
			//if (dl_sp > 5) {
			//	COM_fastled();
			//	dl_sp = 0;
			//}
			break;		

			case 207: //check for reaching miniumum value
				//random kill pixels
								
				if (led_dl[ledcount].r > 1) {
					PRG_reg[2] &= ~(1 << 6);
				}
				else {
					if (led_dl[ledcount].g > 1) {
						PRG_reg[2] &= ~(1 << 6);
					}
					else {
						if (led_dl[ledcount].b > 1) {
							PRG_reg[2] &= ~(1 << 6);
						}
					}

					dl_sp = (led_dl[ledcount].r + led_dl[ledcount].g + led_dl[ledcount].b);
					if (dl_sp == 1 | dl_sp == 2) led_dl[ledcount] = 0x010101;

				}	
				if (ledcount >= led_al) {
				if (bitRead(PRG_reg[2], 6) == true) {
					fase = 210;
				}
				else {
					fase = 206;
					Serial.println("terug naar 206");
				}
				}

				break;

			case 208:
				if (led_dl[ledcount]) {
					teller++;
				}
				


				if (ledcount >= led_al) {
					Serial.println(teller);
					if (teller > (led_al / 15)) {
						PRG_reg[2] &= ~(1 << 6);
						teller = 0;
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
				COM_fastled();
				ledcount = 0;
				PRG_reg[pn]|= (1 << 6); //flag voor bereiken eindwaarde				
				rled[0] = random(0, led_al); 
				rled[1] = random(0, led_al);
				rled[2] = random(0, led_al); 
				randomSeed(analogRead(A5));	//port (pinA5) must be free, not in use
				
				dl_sc = 0;
				dl_s++;
				if (dl_s > 5) dl_s = 0;

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
				dl_sc = 0;
				dl_s = 0;
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
void PRG_lightning() { //Programnummer=4
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
		//Serial.println(mc);		
	}
	
	if (bitRead(PRG_reg[4], 1) == false) { //init start from prg_dl

		Serial.println("start bliksem");

		PRG_reg[4] |= (1 << 1);
		PRG_reg[4] |= (1 << 0);//starten via PRG_reg of modeltijd....
		//start led bepalen
			
		lgtfase = 0;
		time=millis();
		interval = 0;
		mc = 0;
		duur =  random(60, 300); //hoe lang het moet onweren
		Serial.print("Duur: ");
		Serial.println(duur);
		it = random(6000, 12000);

	}
	else { //program verloop
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
				COM_fastled();
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
							PRG_reg[4] &= ~((1 << 1));
							PRG_reg[4] &= ~((1 << 2)); //disable modeltijd start
							Serial.println("stop bliksem");
						}
						else {
							lgtfase = 0;
						}
					}
					COM_fastled();
				break;
			}
		time = millis();
		}
	}
}
void PRG_dld(byte pn) {
	//switches between day and night without sunrise or sunset		
	static unsigned long t = 0;
	static byte lc=0;
	if (millis() - t > 1) {
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
		if (lc > led_al) {
			COM_fastled();
			lc = 0;
			PRG_reg[pn] &= ~(1 << 0); 
			Serial.println("stop dld");
		}
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
	static byte output1=4;
	static byte output2=5;
	
	

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
				out = output2;
				periode = 5000;
				LED_set(output1, 200,0,0);
				LED_set(output2, 200,0,0);
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
				LED_set(out, 0,200,0);
				fase = 3;
				break;

			case 3: //OW oranje
				periode = 4000;
				LED_set(out, 0,0,150);
				fase = 4;
				break;
			case 4:
				//all red
				periode = 5000;
				LED_set(output1, 200,0,0);
				LED_set(output2, 200,0,0);
				fase = 1;
				break;
			}
			//COM_fastled();
		}
	}
}
void PRG_las() { //programma 6
	//simutatie lassen
	static byte out = 6;
	static unsigned long t;
	if (millis() - t > 20) {
		t = millis();
		PRG_reg[6] ^= (1 << 7);
		if (bitRead(PRG_reg[6], 7) == true) {
			LED_set(out, 0, 0, 0);
		}
		else {
			LED_set(out, 255, 255, 255);
		}
	}
}
void PRG_build() { //prg 10
//huizen en gebouwen
	//PRG_reg
	//bit 6 flag for used program part
	//bit 7 = dagnacht status laatste doorloop
	static byte bld_min;
	static int bld_mc; //mc=minute count
	static byte bld_output;
	static byte bld_rdm[6]; //random 'starttijd van programma'

	//minuten teller, vertraagd uitvoering 1x per minuut en onderstaand wordt
	//per modelminuut maar 1x doorlopen, bij volgende doorloop is mt_min verhoogd.
	if (bld_min != mt_min) {
		bld_min = mt_min;
		bld_mc++;
		if (bitRead(PRG_reg[10], 7) != bitRead(COM_reg, 3)) { //dagnacht is gewisseld
			PRG_reg[10] ^= (1 << 7);
			bld_mc = 0;
			//calculate 6 random start numbers
			for (int i = 0; i < 6; i++) {
				bld_rdm[i] = random(0, 30);
			}
		}
		//***************huis1
		//check of program wordt gebruikt en lees current waarde van de leds in
		bld_output = 1; //klein huis1, 1 pixel
		build_use(bld_output);
		if (bitRead(PRG_reg[10], 6) == true) {
			build_h1(bld_mc, bld_rdm[0]);
			LED_go(bld_output);
		}
		//*******************huis 2

		//prg = 2; //klein huis2, 1 pixel

		//***************Herenhuis1
		//tijdens ontwerp even 2e en 3e pixel (1 en 2) en output 2
		//2 pixels, pixels apart
		bld_output = 2; // 1e output
		for (int i = 0; i < 2; i++) {			
			build_use(bld_output + i);
			if (bitRead(PRG_reg[10], 6) == true) {
				build_hh1(bld_mc, bld_rdm[5],1+i);
				LED_go(bld_output+i);
			}
		}
	}
}
void build_use(byte prg) {
	//checks if programpart is in use, loads pixel value, sets flag
	PRG_reg[10] &= ~(1 << 6);//clear flag
	for (int i = 0; i < 32; i++) {
		if (led_vlap[i] == prg) {
			PRG_reg[10] |= (1 << 6); //set flag, program is active
			BLD_clr[0] = led_vl[i].r;
			BLD_clr[1] = led_vl[i].g;
			BLD_clr[2] = led_vl[i].b;
			i = 50;
		}
	}
}
void build_h1(int mnt,int rdm) {
	//klein huis 1 pixel output #1 r(1)=boven g(0)=beneden b(2)=wc, badkamer
	if (bitRead(COM_reg, 3) == true) { //naar nacht

		switch (mnt+rdm) { //mc=minute count, verstreken tijd na wisseling dag/nacht
		case 30:
			BLD_clr[1] = 0xFF;
			break;
		case 80:
			BLD_clr[2] = 0xFF;
			break;
		case 100:
			BLD_clr[0] = 0xFF;
			break;
		case 175:
			BLD_clr[1] = 2;
			break;
		case 190:
			BLD_clr[2] = 2;
			break;
		case 210:
			BLD_clr[0] = 0;
			break;
		}
	}
	else { //naar dag
		switch (mnt-rdm) {
		case 30: //uitschakelen 
			BLD_clr[0] = 100; //beneden aan
			break;
		case 50:
			BLD_clr[2] = 0xFF; //wc
			break;
		case 75:
			BLD_clr[2] = 0; //wc uit
			break;
		case 100:
			BLD_clr[1] = 0xFF; //slaapkamer
			break;
		case 110:
			BLD_clr[0] = 0; //beneden
			break;
		case 120:
			BLD_clr[1] = 0;
			break;
		}
	}	
}

void build_h2() {

}
void build_hh1(int mnt, int rdm,byte pix) {
	//Middelgroothuis 2pixels output (tijdelijk) #2-#3  
	//p1r=gang, garberobe
	//p1g=huiskamer beneden
	//p1b=buitenlicht
	//p2r=zolder
	//p2g=slaapkamer links
	//p2b=WC badkamer
	if (bitRead(COM_reg, 3) == true) {//naar nacht
		switch (mnt + rdm) {
		case 30:
			if (pix == 1)BLD_clr[0] = 0xFF; //gang
			break;
		case 36:
			if (pix == 1)BLD_clr[2] = 0xFF; //buitenlamp
			break;			
		case 42:
			if (pix == 1)BLD_clr[1] = 0xFF; //huiskamer aan
			break;
		case 45:
			if (pix == 1)BLD_clr[0] = 0;//gang uit
			break;
		case 100:
			if (pix == 1)BLD_clr[0] = 0xFF; //gang aan
			break;
		case 105:
			if (pix == 2)BLD_clr[2]= 0xFF; //WC aan
			break;
		case 110:
			if (pix == 2)BLD_clr[0] = 0xFF; //Zolder aan
			break;
		case 115:
			if (pix == 2)BLD_clr[2] = 0; //WC uit
			break;
		case 118:
			if (pix == 1)BLD_clr[0] = 0; //gang uit
			break;
		case 170:
			if (pix == 1)BLD_clr[0] = 0xFF; //gang aan
			break;
		case 175:
			if (pix == 2)BLD_clr[2] = 0xFF; //WC aan
			break;
		case 185:
			if (pix == 2)BLD_clr[2] = 0; //WC uit
			break;
		case 190:
			if (pix == 1)BLD_clr[1] = 2; //huiskamer zwak uit
			break;
		case 195:
			if (pix == 2)BLD_clr[1] = 0xFF; //slaapkamer aan
			break;
		case 198:
			if (pix == 1)BLD_clr[0] = 0; //gang uit
			break;
		case 205:
			if (pix == 2)BLD_clr[1] = 2; //slaapkamer zwak
			break;
		case 230:
			if (pix == 2)BLD_clr[2] = 0xFF; //WC aan
			break;
		case 240:			
			if (pix == 2)BLD_clr[2] = 0; //WC uit
			break;
		case 250:
			if (pix == 2)BLD_clr[0] = 2; //Zolder zwak
		case 355:
			if (pix == 2)BLD_clr[0] = 0xFF; //zolder aan
			break;
		case 360:
			if (pix == 2)BLD_clr[2] = 0xFF; //WC aan
			break;
		case 375:
			if (pix == 2)BLD_clr[2] = 0; //WC uit
			break;
		case 380:
			if (pix == 2)BLD_clr[0] = 2; //zolder zwak
			break;
		case 490:
			if (pix == 2)BLD_clr[0] = 0xFF; //zolder aan
			break;
		case 500:
			if (pix == 2)BLD_clr[2] = 0xFF; //WC aan
			break;
		case 520:
			if (pix == 2)BLD_clr[2] = 0; //WC uit
		break;
		case 530:
			if (pix == 2)BLD_clr[0] = 2; //zolder zwak
			break;
		}
	}
	else { //naar dag
		switch (mnt - (rdm)) {
		case 30:
			break;
		case 60:
			if (pix == 1)BLD_clr[2] = 0;
			break;
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
void zwart(byte led, byte dec, byte mr, byte mg, byte mb, boolean stop) {
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
}
void dndirect(byte st) { //=dag/nacht 0=toggle 1=day 2=night
		switch (st) {
		case 0:
			COM_reg ^= (1 << 3); //bit3 false = day, true is night	
			break;
		case 1:
			COM_reg &= ~(1 << 3);
			break;
		case 2:
			COM_reg |= (1 << 3);
			break;
		}
		PRG_reg[3] |= (1 << 0);//start program 3
		PRG_reg[2] &= ~(1 << 0); //Stop program 2, 
		PRG_reg[2] &= ~(1 << 2); //disable modeltime start				

		if (bitRead(COM_reg, 3) == false) {
			PORTB |= (1 << 5);
			PORTB &= ~(1 << 4);
			PRG_reg[4] &= ~(1 << 0);
			PRG_reg[4] &= ~(1 << 1);
			PRG_reg[4] &= ~(1 << 2);
		}
		else {
			PORTB |= (1 << 4);
			PORTB &= ~(1 << 5);
		}
}
void lightningstart(byte kans) { //start lightningeffect op tijd
//kans....//	0=nooit, 1=altijd, 2=50%, 3=33%,  4=25%	
	static byte temp;	
	if (bitRead(COM_set, 0) == true) { //enable lighting
		
		temp = random(1, kans + 1); 
		if (temp == 1) {
			temp = 0;
			Serial.println("onweer");
			while (temp != mt_zonop) {
				temp++;
			}
			temp = random(1, temp + 1);
			PRG_hr[4] = mt_zononder; //set wakeuptime for lightning
			for (int i = 0; i < temp; i++) {
				PRG_hr[4] = PRG_hr[4] + 1;
				if (PRG_hr[4] > 24)PRG_hr[4] = 1;
			}
			PRG_min[4] = random(0, 59);
			PRG_reg[4] |= (1 << 2); //enable modeltijd start
		}
	}
}
void loop() {
DEK_DCCh();
COM_Clk();
COM_ProgramAssign();
COM_switch();
}
