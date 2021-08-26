#include "MS5837.h"  //library for pressure sensor
#include <Wire.h>

MS5837 sensor;  //creates sensor object

void setup() {
  Serial.begin(9600); //starts serial communication with laptop

  //Set up for the depth sensor
  Serial.println("Starting"); 
  
  Wire.begin();
  while (!sensor.init()) { //if init keeps failing, you'll keep seeing error messages
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL"); //idk what this message is supposed to indicate
    Serial.println("\n\n\n");
    delay(5000);}


}

void loop() {
  sensor.read(); //the hardware gets sampled
  //currDepth = sensor.depth(); //sensor.depth() calls the depth value that just got read.
  Serial.print(sensor.depth());
  Serial.print("\n");
  delay(50);
}
  // put your main code here, to run repeatedly:
