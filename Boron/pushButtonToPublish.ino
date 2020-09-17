//write to D0 pin
int led1 = D0;
//write to D1 when the button is pressed
int buttonClick = D1;

String deviceOwner = "Miggie Hedrick"; 

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
      
   
    
	//publish to the Particle.io cloud
    Particle.publish("Fall Detected",deviceOwner, 3600, PRIVATE); 
    delay(2000); 
  }//end while loop
  
  //turn off led
  digitalWrite(led1, LOW);
    
}

