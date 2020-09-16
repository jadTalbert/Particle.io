//write to D0 pin
int led1 = D0;
//write to D1 when the button is pressed
int buttonClick = D1;

//set up
void setup() {
    pinMode(led1, OUTPUT);
    pinMode(buttonClick, INPUT_PULLDOWN);
}

//infinite loop to continously put voltage on the led, but only lights up when button is pressed
void loop() {

  // To blink the LED, first we'll turn it on...
  digitalWrite(led1, HIGH);
  
  //while loop to continously determine if the button has been pressed. If voltage is on the pin, then publish to the particle cloud
  while(digitalRead(buttonClick) == HIGH) {
      
     //JSONValue outerObj = JSONValue::parseCopy( String("[\"abc\",\"def\",\"xxx\"]"));
     String jsonDataString = String( "{ \"name\":" + String("YourStringHere") + ",\"event\":" + String("YourEventHere") +"}");

	//publish to the Particle.io cloud
    Particle.publish("Fall Detected",jsonDataString, 3600, PRIVATE); 
    delay(2000); 
  }//end while loop
  
  //turn off led
  digitalWrite(led1, LOW);
    
}

