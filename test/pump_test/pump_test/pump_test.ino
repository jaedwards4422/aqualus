
const int OUT3 = 8;
const int OUT4 = 9;
const int ENB = 10; //PWM, ENB pin
void setup() {

  //Set up for pump
  pinMode(OUT3,OUTPUT); // when high, reverse
  pinMode(OUT4,OUTPUT); //when high, forward
  pinMode(ENB, OUTPUT); //PWM controls speed of motor
}

void loop() {
  // test if pump runs. might need to change which pin is high and which is low
  digitalWrite(OUT3,LOW);
  digitalWrite(OUT4,HIGH);
  analogWrite(ENB,255);
  delay(5000);

  digitalWrite(OUT3,LOW); //pause 1sec
  digitalWrite(OUT4,LOW);
  analogWrite(ENB,0);
  delay(1000);

}
