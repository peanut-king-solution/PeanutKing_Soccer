#include <PeanutKingSoccerV4.h>
static PeanutKingSoccerV4 robot = PeanutKingSoccerV4();

void setup() {
  robot.init(); 
  pinMode(S1_P, OUTPUT);
  pinMode(S2_P, OUTPUT);
  pinMode(S3_P, OUTPUT);
  pinMode(S4_P, OUTPUT);

  pinMode(D1_P, INPUT_PULLUP);
  pinMode(D2_P, INPUT_PULLUP);
  pinMode(D3_P, INPUT_PULLUP);
  pinMode(D4_P, OUTPUT);
  pinMode(D5_P, OUTPUT);
  pinMode(D6_P, OUTPUT);

  pinMode(A1_P, INPUT);
  pinMode(A2_P, INPUT);
  pinMode(A3_P, INPUT);
  pinMode(A4_P, INPUT);
}

void loop() {
  analogWrite(S1_P, 60); 
  analogWrite(S2_P, 70); 
  analogWrite(S3_P, 100);
  analogWrite(S4_P, 200);

  digitalWrite(D4_P, HIGH);
  digitalWrite(D5_P, HIGH);
  digitalWrite(D6_P, HIGH);


  Serial.println("DigitalRead:");
  Serial.print(digitalRead(D1_P));
  Serial.print(" ");
  Serial.print(digitalRead(D2_P));
  Serial.print(" ");
  Serial.print(digitalRead(D3_P));
  Serial.print("\n");

  Serial.println("AnalogRead:");
  Serial.print(analogRead(A1_P));
  Serial.print(" ");
  Serial.print(analogRead(A2_P));
  Serial.print(" ");
  Serial.print(analogRead(A3_P));
  Serial.print(" ");
  Serial.print(analogRead(A4_P));
  Serial.print("\n");

  robot.setOnBrdLED(LED_CYAN);
  delay(200);
  digitalWrite(D4_P, LOW);
  digitalWrite(D5_P, LOW);
  digitalWrite(D6_P, LOW);
  
  robot.setOnBrdLED(0,HIGH);
  robot.setOnBrdLED(1,LOW);
  robot.setOnBrdLED(2,LOW);

  delay(200);

}

  
