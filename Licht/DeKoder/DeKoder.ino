/*
 Name:		DeKoder.ino
 Created:	11/28/2017 10:01:32 PM
 Author:	Rob Antonisse
 Very simple DCC decoder for model railroads. 
 Only for accessory decoders. Action and CV programming.

*/

//Temp declaraties
unsigned long Tijd;
int comcount;
int count;
int splitcount;
int falsecount;
int commandcount;

//Declaraties
volatile unsigned long Tperiode; //laatst gemeten tijd 
volatile unsigned int Tduur; //gemeten duur van periode tussen twee interupts
byte countbyte = 0; //counter received bytes
byte DekReg; //register voor de decoder 
byte DekStatus = 0;
byte byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte countPA = 0; //counter for preample

byte BufReg[12]; //registerbyte for 12 command buffers
//bit7= free (false) bit0= CV(true)
byte Buf0[12];
byte Buf1[12];
byte Buf2[12];
byte Buf3[12];
byte Buf4[12];
byte Buf5[12];

void setup() {
	Serial.begin(9600);

DDRB |= (1 << 5);	//pin13
DDRB |= (1 << 4);  //Pin12 als output
bitClear(DDRD, 2); //pin2 input
Tijd = millis();
Tperiode = micros();
//interrupt op PIN2 aanzetten, is INT0	
EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1
} 
ISR(INT0_vect) { //syntax voor een ISR
//isr van PIN2
	//DekReg fase van bit ontvangst
	//bit 0= bitpart ok (1) of failed(0)
	//bit1= truepart 
	//bit2=falsepart
	//bit3= received bit true =true false =false
	//bit4=restart, begin, failed as true

	cli();
	Tduur = (micros() - Tperiode);
	Tperiode = micros();
	if (Tduur > 50) {
		if (Tduur < 62) {
			DekReg |= (1 << 0); //bitSet(DekReg, 0);

			if (bitRead(DekReg, 1) == false) {
				DekReg &= ~(1 << 2); //bitClear(DekReg, 2);
				DekReg |=(1<< 1);
			}
			else { //received full true bit

				DekReg |=(1<< 3);
				BitRX();
				DekReg &= ~(1 << 2);//bitClear(DekReg, 2);
				DekReg &= ~(1 << 1); //bitClear(DekReg, 1);
			}
		}
		else {
			if (Tduur > 106) {

				if (Tduur < 124) { //preferred 118 6us extra space in false bit
					DekReg |= (1 << 0); //bitSet(DekReg, 0);
					if (bitRead(DekReg, 2) == false) {
						DekReg &= ~(1 << 1); //bitClear(DekReg, 1);
						DekReg |= (1 << 2);  //bitSet(DekReg, 2);
					}
					else { //received full false bit
						DekReg &= ~(1 << 3); //bitClear(DekReg, 3);
						BitRX();
						DekReg &= ~(1 << 2);//bitClear(DekReg, 2);
						DekReg &= ~(1 << 1); //bitClear(DekReg, 1);
					}
				}
			}
		}
	}
	sei();
}
void begin() {//runs when bit is corrupted, or command not correct
	//lesscount++;

	countPA = 0;
	countbyte = 0;
	DekReg = 0;
	DekStatus = 0;
	for (int i = 0; i < 6; i++) {
		byteRX[i]=0; //reset receive array
	}
}
void BufCom(boolean CV) { //create command in Buffer

	for (byte i = 0; i < 12; i++) {

		if (bitRead(BufReg[i], 7) == false) {
			BufReg[i] = 0; //clear found buffer
			BufReg[i] |= (1 << 7); //claim buffer
			Buf0[i] = byteRX[0];
			Buf1[i] = byteRX[1];
			Buf2[i] = byteRX[2];

			if (CV == true) {
				BufReg[i] |= (1 << 0); //set for CV
				Buf3[i] = byteRX[3];
				Buf4[i] = byteRX[4];
				Buf5[i] = byteRX[5];
			}
			else {

				Buf3[i] = 0;
				Buf4[i] = 0;
				Buf5[i] = 0;
			}
			i = 15;
		}			
	} //close for loop
} //close void
void BitRX() { //new version
	static byte countbit = 0; //counter received bits
	static byte n = 0;
	DekReg |= (1 << 4);//resets and starts process if not reset in this void
	switch (DekStatus) {
//*****************************
	case 0: //Waiting for preample 
		if (bitRead(DekReg, 3) == true) {
			countPA++;
			if (countPA >12) {
				DekStatus = 1;
				countbit = 0;
				countbyte = 0;
			}
			bitClear(DekReg, 4);
		}
		break;
//*************************
	case 1: //Waiting for false startbit
		if (bitRead(DekReg, 3) == false) { //startbit receive
			countPA = 0;
			DekStatus = 2;
		}
		//if Dekreg bit 3= true no action needed.
		bitClear(DekReg, 4); //correct, so resume process
		break;
//*************************
	case 2: //receiving data
		if (bitRead(DekReg, 3) == true) byteRX[countbyte] |= (1 << (7 - countbit));
		countbit++;
		if (countbit == 8) {
			countbit = 0;
			DekStatus = 3;
			countbyte ++;
		}
		bitClear(DekReg, 4); //correct, so resume process
		break;
//*************************
	case 3: //waiting for separating or end bit
		if (bitRead(DekReg, 3) == false) { //false bit
			DekStatus = 2; //next byte
			if ((bitRead(byteRX[0],6)==false) & (bitRead(byteRX[0],7)==true))bitClear(DekReg, 4); //correct, so resume process	
		}
		else { //true bit, end bit, only 3 byte and 6 byte commands handled by this dekoder
			switch (countbyte) {
			case 3: //Basic Accessory Decoder Packett received
				//check error byte
				if (byteRX[2] = byteRX[0] ^ byteRX[1])BufCom(false);
				break; //6
			case 6 : ///Accessory decoder configuration variable Access Instruction received (CV)
				//check error byte
				if (byteRX[5] = byteRX[0] ^ byteRX[1] ^ byteRX[2] ^ byteRX[3] ^ byteRX[4])BufCom(true);
				break;
			} //close switch bytecount
		}//close bittype
		break;
//***************************************
	} //switch dekstatus
	if (bitRead(DekReg, 4) == true)begin();
}
void DCCh() { //handles incoming DCC commands
	static byte n = 0; //one buffer each passing
	byte temp;
	//Validate new command
	if (bitRead(BufReg[n], 7) == true) { //*
		//Validate basic accessory decoder packet bit7 true, bit 6 false

		//Serial.print("Bytecount 6 :  ");
		//Serial.println(comcount);
		//Serial.print("Bytecount 3 :  ");
		//Serial.println(count);

		
		Serial.print("command...: ");
		Serial.println(commandcount);
		commandcount++;
		Serial.print("buffer= ");
		Serial.print(n);
		Serial.print("  value:  ");
		Serial.print(bitRead(BufReg[n], 7));
		Serial.print(bitRead(BufReg[n], 6));
		Serial.print(bitRead(BufReg[n], 5));
		Serial.print(bitRead(BufReg[n], 4));
		Serial.print(bitRead(BufReg[n], 3));
		Serial.print(bitRead(BufReg[n], 2));
		Serial.print(bitRead(BufReg[n], 1));
		Serial.print(bitRead(BufReg[n], 0));
		Serial.println("");

temp = Buf0[n];
Serial.print(bitRead(temp, 7));
Serial.print(bitRead(temp, 6));
Serial.print(bitRead(temp, 5));
Serial.print(bitRead(temp, 4));
Serial.print(bitRead(temp, 3));
Serial.print(bitRead(temp, 2));
Serial.print(bitRead(temp, 1));
Serial.print(bitRead(temp, 0));
Serial.println("");

temp = Buf1[n];
Serial.print(bitRead(temp, 7));
Serial.print(bitRead(temp, 6));
Serial.print(bitRead(temp, 5));
Serial.print(bitRead(temp, 4));
Serial.print(bitRead(temp, 3));
Serial.print(bitRead(temp, 2));
Serial.print(bitRead(temp, 1));
Serial.print(bitRead(temp, 0));
Serial.println("");

temp = Buf2[n];
Serial.print(bitRead(temp, 7));
Serial.print(bitRead(temp, 6));
Serial.print(bitRead(temp, 5));
Serial.print(bitRead(temp, 4));
Serial.print(bitRead(temp, 3));
Serial.print(bitRead(temp, 2));
Serial.print(bitRead(temp, 1));
Serial.print(bitRead(temp, 0));
Serial.println("");

temp = Buf3[n];
Serial.print(bitRead(temp, 7));
Serial.print(bitRead(temp, 6));
Serial.print(bitRead(temp, 5));
Serial.print(bitRead(temp, 4));
Serial.print(bitRead(temp, 3));
Serial.print(bitRead(temp, 2));
Serial.print(bitRead(temp, 1));
Serial.print(bitRead(temp, 0));
Serial.println("");

temp = Buf4[n];
Serial.print(bitRead(temp, 7));
Serial.print(bitRead(temp, 6));
Serial.print(bitRead(temp, 5));
Serial.print(bitRead(temp, 4));
Serial.print(bitRead(temp, 3));
Serial.print(bitRead(temp, 2));
Serial.print(bitRead(temp, 1));
Serial.print(bitRead(temp, 0));
Serial.println("");


temp = Buf5[n];
Serial.print(bitRead(temp, 7));
Serial.print(bitRead(temp, 6));
Serial.print(bitRead(temp, 5));
Serial.print(bitRead(temp, 4));
Serial.print(bitRead(temp, 3));
Serial.print(bitRead(temp, 2));
Serial.print(bitRead(temp, 1));
Serial.print(bitRead(temp, 0));
Serial.println("");

//clear buffer
BufReg[n] = 0;
Buf0[n] = 0;
Buf1[n] = 0;
Buf2[n] = 0;
Buf3[n] = 0;
Buf4[n] = 0;
Buf5[n] = 0;




//bitClear(BufReg[n], 7); //free buffer
/*
//delay(1000);


	Serial.print("buffer= ");
	Serial.print(n);
	Serial.print("  value:  ");
	Serial.print(bitRead(BufReg[n], 7));
	Serial.print(bitRead(BufReg[n], 6));
	Serial.print(bitRead(BufReg[n], 5));
	Serial.print(bitRead(BufReg[n], 4));
	Serial.print(bitRead(BufReg[n], 3));
	Serial.print(bitRead(BufReg[n], 2));
	Serial.print(bitRead(BufReg[n], 1));
	Serial.print(bitRead(BufReg[n], 0));
	Serial.println("");
	Serial.println("-------------");
	Serial.println("");

*/
	} //*	
	n++;
	if (n > 12)n = 0;
}
void loop() {
	DCCh();
	//if ((millis() - Tijd) > 5000) Tijd = millis();
}
