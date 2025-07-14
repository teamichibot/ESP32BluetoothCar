#include "BluetoothSerial.h"
#include <ESP32Servo.h>

BluetoothSerial SerialBT;

#define IN1 27
#define IN2 26
#define ENA 14
#define SERVO_PIN 13

Servo steeringServo;
String rxValue = "";

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-RC-Car"); // Device name for pairing

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(90); // Center
}

void loop() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    rxValue = String(c);
    Serial.println("Received: " + rxValue);

    if (rxValue == "v") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 200);
    } else if (rxValue == "c") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 200);
    } else if (rxValue == "b") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
    } else if (rxValue == "d") {
      steeringServo.write(120);
    } else if (rxValue == "a") {
      steeringServo.write(70);
    } else if (rxValue == "w") {
      steeringServo.write(90);
    }
  }
}
