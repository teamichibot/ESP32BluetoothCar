# ESP32 RC Car with Bluetooth Classic and Servo Steering

## Description

This project enables you to control a **RC car** using **ESP32** with **Bluetooth Classic**. It uses:
- **L298N motor driver** to control DC motors (forward, backward, and stop)
- **MG996R Servo** for steering (left, right, and center)
- **Bluetooth Classic (Serial)** to control the car via **Serial Bluetooth Terminal** or a custom joystick app on your **iPhone** or **Android**.

## Features
- Control forward, backward, and stop for the car.
- Steering control with a servo motor (left, right, center).
- Remote control via Bluetooth using **Serial Bluetooth Terminal** or a custom joystick app. app : BlueDuino on plasystore

## Components Needed
- **ESP32** development board : https://s.shopee.co.id/5L0uSt61wq
- **L298N Motor Driver** for motor control : https://s.shopee.co.id/qYV6h4uyD
- **Step Down LM2596** : https://s.shopee.co.id/5fdkrhWMVN
- **3S Li-ion Battery** (11.1V) for powering the motors and ESP32 : https://s.shopee.co.id/6VCrr98ald
- **18650 3s Battery Holder** : https://s.shopee.co.id/9ADd2D3PGu
- **BMS 3s 20A** : https://s.shopee.co.id/8pamddXUrp
- **4WD Car Chassis with 25mm DC motor + MG996r Servo** : https://s.shopee.co.id/1BBLVd7bI4

## Wiring Diagram

### Connections:

- **ESP32** Pin Configuration:
  - `IN1` → GPIO 26 (Motor A)
  - `IN2` → GPIO 27 (Motor A)
  - `ENA` → GPIO 14 (PWM motor control)
  - `Servo` → GPIO 13 (Steering control)

- **L298N Motor Driver:**
  - `VCC` → 12V power supply (from the battery)
  - `GND` → Common ground (to ESP32, L298N, and servo)
  - `OUT1, OUT2` → Connected to the DC motor
  - `ENA` → Connected to PWM pin (GPIO 14)

- **Servo (MG996R):**
  - `VCC` → 5V power (VOUT+ Step Down)
  - `GND` → (VOUT- Step Down)
  - Control signal → GPIO 13

## Power Supply

1. **ESP32**: Can be powered via the **5V pin** (from L298N 5V or a separate 5V buck converter).
2. **Servo**: Powered by a **separate 5V step-down converter** from a 12V battery (not from ESP32 directly).
3. **L298N**: Powered by a **12V Li-ion battery** (3S 18650 cells).

Make sure the **ground (GND)** of the battery, ESP32, L298N, and Servo are all connected to a **common ground**.

## Code

```cpp
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

BluetoothSerial SerialBT;

#define IN1 25
#define IN2 27
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

    if (rxValue == "F") {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 200);
    } else if (rxValue == "B") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 200);
    } else if (rxValue == "S") {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
    } else if (rxValue == "L") {
      steeringServo.write(120);
    } else if (rxValue == "R") {
      steeringServo.write(60);
    } else if (rxValue == "C") {
      steeringServo.write(90);
    }
  }
}
