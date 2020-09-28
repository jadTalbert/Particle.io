#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduinoJson.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//write to D5 pin
int led1 = D5;
//write to D6 when the button is pressed
int buttonClick = D6;

//counter to track acceleromter health
int accResetCounter = 0;

//owner of this device
String deviceOwner = "Miggie Hedrick"; 

//set up
void setup() {

  pinMode(led1, OUTPUT);
  pinMode(buttonClick, INPUT_PULLDOWN);

  //begin serial output
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

  //init the sensor
  bno.begin();

  //delay for 1000ms
  delay(1000);
  
  //use the external temp crystal(e.g. NOT the one onboard the IMU)
  bno.setExtCrystalUse(true);

}

 //void(* resetFunc) (void) = 0;

//infinite loop to continously put voltage on the led, but only lights up when button is pressed
void loop() {

  // To blink the LED, first we'll turn it on...
  digitalWrite(led1, HIGH);
  
  //while loop to continously determine if the button has been pressed. If voltage is on the pin, then publish to the particle cloud
  while(digitalRead(buttonClick) == HIGH) {
	  //publish to the Particle.io cloud
    Particle.publish("Fall Detected",deviceOwner, 3600, PRIVATE); 
    //delay(2000); 
  }//end while loop
  
  //turn off led
  digitalWrite(led1, LOW);


  //////begin accelerometer code////////////////

  // allocate the memory for the document
  const size_t CAPACITY = JSON_OBJECT_SIZE(3);
  StaticJsonDocument<CAPACITY> doc;
  // create an object
  JsonObject object = doc.to<JsonObject>();
  
  //create handles to each respective object(e.g. accelerometer, gyroscope and magnetometer)
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);


 
  //if all three dimensions are reporting zero for an extended period of time, something is wrong with the acclerometer
  // this behavior was found when dropping from a height of 10-15 inches when device when attached to the bread board
  if(acc.x() == 0 && acc.y() == 0 && acc.z() == 0){
     accResetCounter = accResetCounter + 1;
  }
  else{//make sure the  counter doesn't leak over time
    accResetCounter = 0;
  }

  //if the device reports a count >= 50, then reset the accelerometer
  if(accResetCounter >= 100){
    accResetCounter = 0;
    Serial.println("Accelerometer no longer recording properly. Resetting now!");
    System.reset();//reset the device if an anomoly is detected(e.g. device is dropped and the accelerometer fails to register data)
    delay(2000);
  }


  //create the map for each respective dimension for accleration
  object["x-axis"] = acc.x();
  object["y-axis"] = acc.y();
  object["z-axis"] = acc.z();

  // serialize the object and send the result to Serial
  serializeJson(doc, Serial);
  
  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting next data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  //////end accelerometer code//////////////
    
}

