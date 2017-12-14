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

byte commandfree;




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
	if (bitRead(DekReg, 0) == false) failed();

	PINB |= (1 << 5); //toggle pin 13
	sei();
}
void failed() {//runs when bit is corrupted, or command not correct
	lesscount++;
	DekReg = 0;
	DekStatus = 0;
	for (int i = 0; i < 6; i++) {
		byteRX[i]=0; //reset receive array
	}
}
void BitRX() { //handles recieved bits
	static byte countPA = 0; //counter for preample
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0; //counter received bytes

	switch (DekStatus) {
	
	case 0: //Waiting for preample 
		if (bitRead(DekReg, 3) == true) {
			countPA++;
			if (countPA > 10) {
				
				DekStatus = 1;
				countbit = 0;
				countbyte = 0;
				countPA = 0; //reset preamplecounter for next command

				for (int i = 0; i < 6; i++) {
					byteRX[i] = 0; //reset receive array
				}

			}
		}
		else {
			failed(); //received zero bit, not allowed in Dekstatus 0
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
			if (countbyte > 5) failed(); //this command cannot be handled by this decoder
			
		}
		break;

	case 3: //waiting for separating or end bit
		if (bitRead(DekReg, 3) == false){ //false bit
			DekStatus = 2;
		}
		else { //true bit
			if (countbyte > 2) {
				//put received data in command buffer, exeption for idle packet do not store this in buffer 255-0-255

			}
			else { //not yet 3 bytes received, is minimum.
				failed(); //reset all data, start over
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

// the loop function runs over and over again until power down or reset
void loop() {

	if ((millis() - Tijd) > 3000) { //knipperlichie op pin 12
		PINB |= (1 << 4);  
		Tijd = millis();
		//getelde waardes tonen

		Serial.print("Aantal true=  ");
		Serial.println(truecount);
		Serial.print("Aantal false=  ");
		Serial.println(falsecount);
		
		
		Serial.print("Aantal failed=  ");
		Serial.println(lesscount);
		Serial.println("");
/*

		Serial.print("Aantal ertussenin=  ");
		Serial.println(splitcount);
		Serial.print("Aantal meer=  ");
		Serial.println(morecount);
		
*/

		//counters resetten
		lesscount = 0;
		truecount = 0;
		splitcount = 0;
		falsecount = 0;
		morecount = 0;

	}


}
