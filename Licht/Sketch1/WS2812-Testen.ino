/*
 Name:		Sketch1.ino
 Created:	11/26/2017 10:28:59 PM
 Author:	gebruiker
*/

// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(13, OUTPUT);
}

// the loop function runs over and over again until power down or reset
void loop() {
  //dit is een test van de repos
	if (digitalRead(13) == HIGH) {
		digitalWrite(13, LOW);
	}
	else {
		digitalWrite(13, HIGH);
	}
	delay(50);

}
