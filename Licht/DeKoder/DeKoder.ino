/*
 Name:		DeKoder.ino
 Created:	11/28/2017 10:01:32 PM
 Author:	Rob Antonisse
*/

//Temp declaraties
unsigned long Tijd;
int truecount;
int falsecount;
int nullcount;
//Declaraties
unsigned long Tperiode; //laatst gemeten tijd 
unsigned int Tduur; //gemeten duur van periode tussen twee interupts


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
	cli();
	Tduur = (micros() - Tperiode);
	Tperiode = micros();

	switch (Tduur) {
		case 54: case  55: case  56: case 57: case 58:
//true periode (1/2)
		truecount++;
		break;
	case 110: case 111: case 112: case 113: case 114:
//false periode (1/2)
		falsecount++;

		break;

	default:
		nullcount++;
		break;
	}



	PINB |= (1 << 5); //toggle pin 13
	
	sei();
}


// the loop function runs over and over again until power down or reset
void loop() {

	if ((millis() - Tijd) > 2000) { //knipperlichie op pin 12
		PINB |= (1 << 4);  
		Tijd = millis();
		//getelde waardes tonen

		Serial.print("Aantal true=  ");
		Serial.println(truecount);
		Serial.print("Aantal false=  ");
		Serial.println(falsecount);
		Serial.print("Aantal null=  ");
		Serial.println(nullcount);
		Serial.println("");

		//counters resetten
		truecount = 0;
		falsecount = 0;
		nullcount = 0;

	}


}
