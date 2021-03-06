
const int OUT1 = 2;
const int OUT2 = 3;
const int ENA = 5; //PWM, ENA pin
void setup() {

  //Set up for motors
  pinMode(OUT1,OUTPUT); // when high, reverse
  pinMode(OUT2,OUTPUT); //when high, forward
  pinMode(ENA, OUTPUT); //PWM controls speed of motor
}

void loop() {
  // test if motor runs. first backwards, pause 1s, then forwards.
  digitalWrite(OUT1,HIGH);
  digitalWrite(OUT2,LOW);
  analogWrite(ENA,255);
  delay(2000);

  digitalWrite(OUT1,LOW); 
  digitalWrite(OUT2,LOW);
  analogWrite(ENA,0);
  delay(1000);

  digitalWrite(OUT1,LOW);
  digitalWrite(OUT2,HIGH);
  analogWrite(ENA,255);
  delay(2000);

  digitalWrite(OUT1,LOW); 
  digitalWrite(OUT2,LOW);
  analogWrite(ENA,0);
  delay(1000);
}
