void setup()
{

  /* add setup code here */
	pinMode(13, OUTPUT);
}

void loop()
{
  /* add main program code here */
	if (digitalRead(13)== HIGH) {
		digitalWrite(13,LOW);
	}
	else {
		digitalWrite(13, HIGH);
	}
	delay (500);
	//dit is een change



}
