/*
 Name:		DeKoder.ino
 Created:	11/28/2017 10:01:32 PM
 Author:	Rob Antonisse
*/

//Temp declaraties
unsigned long Tijd;
int lesscount;
int truecount;
int splitcount;
int falsecount;
int morecount;

//Declaraties
unsigned long Tperiode; //laatst gemeten tijd 
unsigned int Tduur; //gemeten duur van periode tussen twee interupts

byte DekReg; //register voor de decoder 
byte DekStatus = 0;
byte byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte countPA = 0; //counter for preample

byte BufReg[12]; //registerbyte for 12 command buffers
byte Buf0[12];
byte Buf1[12];
byte Buf2[12];
byte Buf3[12];
byte Buf4[12];
byte Buf5[12];


void setup() {
	Serial.begin(9600);


sei(); //enable interupts
DDRB |= (1 << 5);	//pin13
DDRB |= (1 << 4);  //Pin12 als output
bitClear(DDRD, 2); //pin2 input
Tijd = millis();
Tperiode = micros();


//interrupt op PIN2 aanzetten, is INT0
	//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	bitSet(EICRA, 0);
	bitClear(EICRA, 1);
	//bitClear(EICRA, 0);
	//EIMSK – External Interrupt Mask Register bit0 INT0 > 1
	bitSet(EIMSK, INT0);
	//EIFR – External Interrupt Flag Register ? volgens mij niks mee nodig bit0 is weer de INT0 van pin2
	//

}
 
ISR(INT0_vect) { //syntax voor een ISR
//isr van PIN2
	//DekReg fase van bit ontvangst
	//bit 0= bitpart ok (1) of failed(0)
	//bit1= truepart 
	//bit2=falsepart
	//bit3= received bit true =true false =false
	cli();

	Tduur = (micros() - Tperiode);
	Tperiode = micros();
	bitClear(DekReg, 0);

	if (Tduur > 46) {

		if (Tduur < 70) {
			bitSet(DekReg, 0);

			if (bitRead(DekReg, 1) == false) {
				bitClear(DekReg, 2);
				bitSet(DekReg, 1);
			}
			else { //received full true bit

				bitSet(DekReg, 3);
				BitRX();
				bitClear(DekReg, 2);
				bitClear(DekReg, 1);
			}
		}
		else {
			if (Tduur > 104) {

				if (Tduur < 130) {
					bitSet(DekReg, 0);
					if (bitRead(DekReg, 2) == false) {
						bitClear(DekReg, 1);
						bitSet(DekReg, 2);
					}
					else { //received full false bit
						bitClear(DekReg, 3);
						BitRX();
						bitClear(DekReg, 2);
						bitClear(DekReg, 1);
					}
				}
			}
		}
	}
	if (bitRead(DekReg, 0) == false) begin();

	PINB |= (1 << 5); //toggle pin 13
	sei();
}
void begin() {//runs when bit is corrupted, or command not correct
	lesscount++;
	countPA = 0;
	DekReg = 0;
	DekStatus = 0;
	for (int i = 0; i < 6; i++) {
		byteRX[i]=0; //reset receive array
	}
}
void BitRX() { //handles recieved bits
	
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0; //counter received bytes
	byte n = 0;

	switch (DekStatus) {
	
	case 0: //Waiting for preample 
		if (bitRead(DekReg, 3) == true) {
			countPA++;
			if (countPA > 10) {
				
				DekStatus = 1;
				countbit = 0;
				countbyte = 0;

				for (int i = 0; i < 6; i++) {
					byteRX[i] = 0; //reset receive array
				}

			}
		}
		else {
			begin(); //received false bit, not allowed in Dekstatus 0
		}
		break;

	case 1: //Waiting for false startbit
		if (bitRead(DekReg, 3) == false) { //startbit receive
			DekStatus = 2;
		}
//if Dekreg bit 3= true no action needed.
		break;

	case 2: //receiving data
		if (bitRead(DekReg, 3) == true) bitSet(byteRX[countbyte], countbit);
		countbit++;
		if (countbit == 8) {
			countbit = 0; 
			DekStatus = 3;
			
			countbyte ++; //bij teveel ontvangen bytes...? hier uitspringen?
			if (countbyte > 5) begin(); //this command cannot be handled by this decoder
			
		}
		break;

	case 3: //waiting for separating or end bit
		if (bitRead(DekReg, 3) == false){ //false bit
			DekStatus = 2;
		}
		else { //true bit
			if (countbyte > 2) { //***
				//put received data in command buffer, exeption for idle packet do not store this in buffer 255-0-255
				if (byteRX[0] == B11111111) { 
					//idl packed or broadcast command, no handling needed
					begin(); //start over
				}
				else { 
				//bufreg[12] and buf0~buf5
				//find free commandbuffer

				for (int i; i < 12; i++) { //mind, when no free buffer is found, command wil always go to buffer 11
					if (bitRead(BufReg[i],7) == false) {	
						n = i;
						break;
					}
				}
//n=free buffer
				//claim buffer
				BufReg[n] = countbyte; //qnty of bytes
				bitSet(BufReg[n], 7);
				//copy to buffer
				Buf0[n] = byteRX[0];
				Buf1[n] = byteRX[1];
				Buf2[n] = byteRX[2];
				Buf3[n] = byteRX[3];
				Buf4[n] = byteRX[4];
				Buf5[n] = byteRX[5];
				begin(); //restart process
				}
			} //***
			else { //not yet 3 bytes received, is minimum.
				begin(); //reset all data, start over
			}
		}
	}

	switch (bitRead(DekReg, 3)) {
	case true:
		truecount++;
		break;
	case false:
		falsecount++;
		break;
	}

}

void DCCh() { //handles incoming DCC commands
	static byte n = 0; //one buffer each passing
	byte temp;
	//Validate new command
	if (bitRead(BufReg[n], 7) == true) { //*
		//Validate basic accessory decoder packet bit7 true, bit 6 false
		

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
Serial.println("");
Serial.println("");




	temp = temp >> 6;

	if (temp == 2) {
	PINB |= (1 << 4); //toggle pin 12	
	}	

	bitClear(BufReg[n], 7); //free buffer
	} //*	
	n++;
	if (n > 12)n = 0;
}

// the loop function runs over and over again until power down or reset
void loop() {
	DCCh();
/*
	if ((millis() - Tijd) > 5000) {
		for (int i; i < 12; i++) {
			if (bitRead(BufReg[i], 7) == true) {
			Serial.println(Buf0[i]);
			Serial.println(Buf1[i]);
			Serial.println(Buf2[i]);
			Serial.println(Buf3[i]);
			Serial.println(Buf4[i]);
			Serial.println(Buf5[i]);
			Serial.println("");
			bitClear(BufReg[i], 7);
			}
			

		}
	}
*/
}
