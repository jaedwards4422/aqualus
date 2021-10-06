const int OUT1 = 2; //pins for motor
const int OUT2 = 3;
const int ENA = 5; //PWM, ENA pin

const int OUT3 = 8; //pins for pump
const int OUT4 = 9;
const int ENB = 10; //PWM, ENB pin
void setup() {

  //Set up for motors
  pinMode(OUT1,OUTPUT); // when high, reverse
  pinMode(OUT2,OUTPUT); //when high, forward
  pinMode(ENA, OUTPUT); //PWM controls speed of motor

  //Set up for pump
  pinMode(OUT3,OUTPUT); // when high, reverse
  pinMode(OUT4,OUTPUT); //when high, forward
  pinMode(ENB, OUTPUT); //PWM controls speed of pump
}

void loop() {
  // test if motor runs. first backwards, pause 1s, then forwards.
  digitalWrite(OUT1,HIGH);
  digitalWrite(OUT2,LOW);
  analogWrite(ENA,255);
  delay(1000);

  digitalWrite(OUT1,LOW); 
  digitalWrite(OUT2,LOW);
  analogWrite(ENA,0);
  delay(1000);

  digitalWrite(OUT1,LOW);
  digitalWrite(OUT2,HIGH);
  analogWrite(ENA,255);
  delay(1000);

  digitalWrite(OUT1,LOW); 
  digitalWrite(OUT2,LOW);
  analogWrite(ENA,0);
  delay(1000);

  // test if pump runs. might need to change which pin is high and which is low
  digitalWrite(OUT3,LOW);
  digitalWrite(OUT4,HIGH);
  analogWrite(ENB,255);
  delay(3000);

  digitalWrite(OUT3,LOW); //pause 1sec
  digitalWrite(OUT4,LOW);
  analogWrite(ENB,0);
  delay(1000);

}
