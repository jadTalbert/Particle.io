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

// allocate the memory for the JSON document outside of the loop function for performance and memory reasons
const size_t CAPACITY = JSON_OBJECT_SIZE(3);
StaticJsonDocument<CAPACITY> doc;
// create an object
JsonObject object = doc.to<JsonObject>();


//set up
void setup() {

  pinMode(led1, OUTPUT);
  pinMode(buttonClick,INPUT_PULLDOWN);

  //begin serial output
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

  //init the sensor
  bno.begin();
  //delay for 1000ms
  delay(2000);
  //use the external temp crystal(e.g. NOT the one onboard the IMU)
  bno.setExtCrystalUse(true);

}


//infinite loop to continously put voltage on the led, but only lights up when button is pressed
void loop() {

  // To blink the LED, first we'll turn it on...
  digitalWrite(led1, HIGH);
  
  //while loop to continously determine if the button has been pressed. If voltage is on the pin, then publish to the particle cloud
  while(digitalRead(buttonClick) == HIGH) {
    Serial.println("button was clicked");
	  //publish to the Particle.io cloud
    Particle.publish("Fall Detected",deviceOwner, 3600, PRIVATE); 
    delay(2000); 
  }//end while loop


  //////begin accelerometer code////////////////
  
  //create handles to each respective object(e.g. accelerometer, gyroscope and magnetometer)
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);


   uint8_t system, gyros, accel, magn = 0;
   bno.getCalibration(&system, &gyros, &accel, &magn);

  //force the callibration to avoid the device from  misbehaving
  //there are instances where the BNO055 does not want to callibrate itself, so this helps in those situations
  if(system != 3 && accel != 3){
    Serial.println("waiting on calibration...");
    adafruit_bno055_offsets_t calibrationData;
    calibrationData.accel_offset_x = 23085;
    calibrationData.accel_offset_y = 13;
    calibrationData.accel_offset_z = 40;
    calibrationData.gyro_offset_x = 23215;
    calibrationData.gyro_offset_y = 13;
    calibrationData.gyro_offset_z = -27864;
    calibrationData.mag_offset_z = 40;
    calibrationData.mag_offset_x = 23215;
    calibrationData.mag_offset_y = 13;
    calibrationData.accel_radius = 8193;
    calibrationData.mag_radius = -7496;
    bno.setSensorOffsets(calibrationData);
    Serial.println("Calibration Complete! Moving on.");
  }


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
    //System.reset();//reset the device if an anomoly is detected(e.g. device is dropped and the accelerometer fails to register data)
  }

  //x,y,z acceleration calculated by this algo -> https://www.hindawi.com/journals/js/2015/452078/
  float xSquared = acc.x() * acc.x();
  float ySquared = acc.y() * acc.y();
  float zSquared = acc.z() * acc.z();
  float accSum = sqrtf(xSquared + ySquared + zSquared);

  float numerator = sqrtf(xSquared + zSquared);
  float denominator = acc.x();
  float angularVar = 180/3.1415;

  //calculate the angle(theta) -> https://www.hindawi.com/journals/jam/2014/896030/
  float theta = atan(numerator/denominator) * angularVar;

  float _2g = 9.81 * 2.5;
  bool fallDetected  = false;

  if(abs(theta) >= 65 && accSum >= _2g){
   fallDetected = true;
    Serial.println("<----------------------FALL DETECTED---------------------------->");
  }


  //build the map for both x,y,z coords and angular accleration 
  object["acc"] = accSum;
  object["theta"] = theta;
  object["fallDetected"] = fallDetected;
 

  // serialize the object and send the result to Serial
  serializeJson(doc, Serial);


  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting next data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  //////end accelerometer code//////////////
    
}

void showCalibrationValues(){
    adafruit_bno055_offsets_t calibrationData;
    bno.getSensorOffsets(calibrationData);
    Serial.println("//----Add this code to your setup funciton, after bno is initialized----");
    Serial.println("adafruit_bno055_offsets_t calibrationData;");
    Serial.print("calibrationData.accel_offset_x = "); Serial.print(calibrationData.accel_offset_x); Serial.println(";");
    Serial.print("calibrationData.accel_offset_y = "); Serial.print(calibrationData.accel_offset_y); Serial.println(";");
    Serial.print("calibrationData.accel_offset_z = "); Serial.print(calibrationData.accel_offset_z); Serial.println(";");
    Serial.print("calibrationData.gyro_offset_x = "); Serial.print(calibrationData.gyro_offset_x); Serial.println(";");
    Serial.print("calibrationData.gyro_offset_y = "); Serial.print(calibrationData.gyro_offset_y); Serial.println(";");
    Serial.print("calibrationData.gyro_offset_z = "); Serial.print(calibrationData.gyro_offset_z); Serial.println(";");
    Serial.print("calibrationData.mag_offset_z = "); Serial.print(calibrationData.accel_offset_z); Serial.println(";");
    Serial.print("calibrationData.mag_offset_x = "); Serial.print(calibrationData.gyro_offset_x); Serial.println(";");
    Serial.print("calibrationData.mag_offset_y = "); Serial.print(calibrationData.gyro_offset_y); Serial.println(";");
    Serial.print("calibrationData.accel_radius = "); Serial.print(calibrationData.accel_radius); Serial.println(";");
    Serial.print("calibrationData.mag_radius = "); Serial.print(calibrationData.mag_radius); Serial.println(";");
    Serial.println("bno.setSensorOffsets(calibrationData);"); 
    
}

