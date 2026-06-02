#include <Servo.h>

#define IN1 8
#define IN2 9
#define ENA 10

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  servo1.attach(5);
  servo2.attach(6);

  analogWrite(ENA, 255); // Full speed

  Serial.println("Commands:");
  Serial.println("F = Forward");
  Serial.println("R = Reverse");
  Serial.println("S = Stop");
  Serial.println("A/B/C = Servo1 0/90/180");
  Serial.println("D/E/G = Servo2 0/90/180");
}

void loop() {
  if (Serial.available()) {

    char cmd = Serial.read();

    switch(cmd) {

      case 'F':
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        Serial.println("Forward");
        break;

      case 'R':
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        Serial.println("Reverse");
        break;

      case 'S':
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        Serial.println("Stop");
        break;

      case 'A':
        servo1.write(0);
        Serial.println("Servo1 = 0");
        break;

      case 'B':
        servo1.write(90);
        Serial.println("Servo1 = 90");
        break;

      case 'C':
        servo1.write(180);
        Serial.println("Servo1 = 180");
        break;

      case 'D':
        servo2.write(0);
        Serial.println("Servo2 = 0");
        break;

      case 'E':
        servo2.write(90);
        Serial.println("Servo2 = 90");
        break;

      case 'G':
        servo2.write(180);
        Serial.println("Servo2 = 180");
        break;
    }
  }
}