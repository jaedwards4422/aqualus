void setup() {

  //Set up for motors
  pinMode(8,OUTPUT); // when high, reverse
  pinMode(9,OUTPUT); //when high, forward
  pinMode(10, OUTPUT); //PWM
}

void loop() {
  // test if motor runs. first backwards then forwards.
  digitalWrite(8,HIGH);
  digitalWrite(9,LOW);
  //analogWrite(10,255);
  delay(2000);

  digitalWrite(8,LOW);
  digitalWrite(9,HIGH);
  //analogWrite(10,255);
  delay(2000);

}
