
#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;
double desired_depth = 1; //meters
double currDepth = 0;
double motorError = 1; //allowable error in measurement
double pumpError = 1; //allowable error in measurement


byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin       = 2;

// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 8;

volatile byte pulseCount;  

double flowRate;
double flowRateActual;
double flowLitres;
double totalLitres;

unsigned long oldTime4; //used to know if pipe is clogged
unsigned long oldTime3; //used to know if pump is stable at right depth
unsigned long oldTime2; //used to know if should adjust pump pwm
unsigned long oldTime; //used to get period of flow rate reading
unsigned long currTime; //used for period of flow rate reading
unsigned long period;
unsigned long startTime; //used to know if should switch valves

double desired_flow_rate = 13;  // L/min

// for PID
double prev_error=0.0; // L/min
double I_acc=0.0;  // L/min

// PID gains:
double kp = 40.3;
double kd = 0.0;
double ki = 7.123;

double pump_pwm_input = 0.0;
double motor_pwm_input = 0.0;

double oldValue = 0; //old value used in low pass filter1
double weight = 39.32; //weight used in low pass filter

boolean canPump = false; //lets the system know when at the proper depth
boolean emergencyStop = false; //lets the system know when at the proper depth
boolean justStarted = true; //lets system know that it just started pumping
boolean swap = true;
boolean justEnteredDepth = true; //lets system know that it just started pumping
boolean donePumping = false;

int collectionAmount = 10; //number of liters to pump

void setup() {
  //Set up for solenoid valves
  pinMode(5,OUTPUT); //pin used for solenoid filter
  pinMode(3,OUTPUT); //pin used for solenoid cleaning
  //Set up for motors
  pinMode(8,OUTPUT); // when high, reverse
  pinMode(9,OUTPUT); //when high, forward
  pinMode(11,OUTPUT); //PWM signal
  TCCR1B = TCCR1B & B11111000 | B00000001; //set PWM frequency to 31372.55 Hz

  //Set up for the pumps and flow rate sensor
  pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pinMode(10,OUTPUT); //Used for PWM

  
  // attach the channel to the GPIO to be controlled

  flowRate = 0.0;
  flowLitres   = 0;
  totalLitres  = 0;
  oldTime = 0;
  oldTime2 = 0;
  oldTime3 = 0;

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(sensorInterrupt, periodReader, FALLING);
  
  Serial.begin(9600);

  //Set up for the depth sensor
  Serial.println("Starting");
  
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {

  if(!canPump || (millis() - oldTime3 < 10000) || emergencyStop || donePumping){
    pump_pwm_input = 0;
  }
  analogWrite(11, pump_pwm_input); //set the pump pwm
  // Update pressure and temperature readings
  sensor.read();
  currDepth = sensor.depth();
  Serial.print(currDepth);
  Serial.print(" ");
  
   if((currDepth > desired_depth - pumpError) &&  (currDepth < desired_depth + pumpError )){
    canPump = true;
    if(justEnteredDepth){
      oldTime3 = millis();
    }
    justEnteredDepth = false;
  }else{
    canPump = false;
    justEnteredDepth = true;
  }

  //makes sure motor is always trying to bring the pump to the desired depth
  if(currDepth < desired_depth - motorError){
      digitalWrite(8,HIGH);
      digitalWrite(9,LOW);
      analogWrite(10,255);
  }else if( currDepth > desired_depth + motorError){
      digitalWrite(8,LOW);
      digitalWrite(9,HIGH);
      analogWrite(10,255);
  }else{
      analogWrite(10,0);
  }
  //Serial.println(flowRate);
  if((millis() - oldTime2) > 500){    // Only process counters once per second
       pump_pwm_input = PID_controller(flowRate);
       pump_pwm_input = min(255, max(0,pump_pwm_input));
       oldTime2 = millis();
       // Divide the flow rate in litres/minute by 30 to determine how many litres have    
       // passed through the sensor in this 1 second interval
       flowLitres = (flowRate / 120);
//    
      // Add the millilitres passed in this second to the cumulative total
       totalLitres += flowLitres; 
  }

  if(totalLitres > collectionAmount){
    donePumping = true;
    desired_depth = .1;
  }

  

  //controlling flow direction section
  if(canPump && justStarted){
    startTime = millis();
    justStarted = false;
  }

  if((millis() - startTime > 12000) && swap){ 
    analogWrite(5,255);
    analogWrite(3,0);
    startTime = millis();
    swap = false;
  }else if(millis() - startTime > 12000){
    analogWrite(3,255);
    analogWrite(5,0);
    startTime = millis();
    swap = true;
  }

  //clog safety code
  if(flowRateActual == 0){
    oldTime4 = millis();
  }
  if((millis()-oldTime4 > 10000) && (pump_pwm_input > 0)){
    emergencyStop = true;
  }
}

double PID_controller(double curr_flow_rate) {
  
  // calculate current error
  double curr_error = desired_flow_rate - curr_flow_rate;
  // calculate derivative term (e_t - e_{t-1})
  double deriv_term = curr_error - prev_error;
  // set for next time
  prev_error = curr_error;

  // sum PID terms
  double new_u = kp * curr_error + kd * deriv_term + ki * I_acc;

  // update integral term but limit it by making it die out
  I_acc = 0.99 * I_acc + curr_error;

  return new_u;
}

/*
Insterrupt Service Routine
 */

double calculateFlowRate(unsigned long period){
   double x = (double) period;
   double p1 = 2.422e-07;
   double p2 = -0.006526;
   double p3 = 55.41;
   double flowRate = p1*sq(x) + p2*x + p3;
   if(flowRate > 20){
      return 0;
   }else{
      return flowRate;
   }
}

double lowPassFilter(double newValue){
  oldValue = (weight*oldValue + newValue) / (weight+1);
  return oldValue;
}

void periodReader(){
  currTime = micros();
  period = currTime - oldTime;
  flowRateActual = calculateFlowRate(period);
  flowRate = lowPassFilter(flowRateActual);
  oldTime = currTime;
}
