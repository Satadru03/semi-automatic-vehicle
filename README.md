Here's the complete README file for your GitHub repository titled "semi-automatic-vehicle."

---

# Semi-Automatic Vehicle

This repository contains the code for a semi-automatic vehicle project that is controlled using hand gestures. The project uses an MPU6050 sensor for gesture detection and communicates commands to a vehicle via Bluetooth. The vehicle can detect obstacles and adjust its movement accordingly.

## Table of Contents

- [Project Overview](#project-overview)
- [Gesture Transmitter](#gesture-transmitter)
- [Car Receiver](#car-receiver)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)

## Project Overview

The semi-automatic vehicle project is designed to be controlled by hand gestures. It consists of two main components:
1. **Gesture Transmitter**: Detects gestures using an MPU6050 sensor and sends corresponding commands via Bluetooth.
2. **Car Receiver**: Receives commands from the gesture transmitter and controls the car's movement. The car is equipped with obstacle detection using an ultrasonic sensor to prevent collisions.

## Gesture Transmitter

The `gesture_transmitter` code detects specific gestures using the MPU6050 accelerometer and gyroscope sensor. It then transmits corresponding commands via Bluetooth to the car receiver.

### Recognized Gestures and Commands

- **Forward (F):** Roll between 90° and 131°, Pitch between -11° and 15°.
- **Backward (B):** Roll between -121° and -90°, Pitch between -15° and 10°.
- **Left (L):** Roll between -195° and -140°, Pitch between -90° and -50°.
- **Right (R):** Roll between -135° and -70°, Pitch between 65° and 100°.
- **Jump (J):** Roll between 110° and 150°, Pitch between -100° and -35°.
- **Idle (I):** Roll between 100° and 125°, Pitch between 30° and 110°.
- **Kick (K):** Roll between -125° and -90°, Pitch between 30° and 80°.
- **Move (M):** Roll between -110° and -65°, Pitch between -75° and -35°.
- **Stop (S):** Default command when no specific gesture is detected.

### Code

```cpp
#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

MPU6050 mpu;
SoftwareSerial bluetoothSerial(0, 1);

float roll, pitch;
const float rad_to_deg = 57.2958;
char out;

const float ACCEL_SCALE = 8192.0;
const float GYRO_SCALE = 65.5;

unsigned long prevTime;
float deltaTime;

void setup() {
    Serial.begin(9600);
    bluetoothSerial.begin(9600);
    Wire.begin();
    mpu.initialize();

    if (mpu.testConnection())
        Serial.println("MPU6050 connection successful");
    else
        Serial.println("MPU6050 connection failed");

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    prevTime = millis();
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long currentTime = millis();
    deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    calculateAngles(ax, ay, az, gx, gy);
    out = determineOutput(roll, pitch);

    Serial.println(out);
    bluetoothSerial.println(out);

    delay(200);
}

void calculateAngles(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy) {
    float ax_g = ax / ACCEL_SCALE;
    float ay_g = ay / ACCEL_SCALE;
    float az_g = az / ACCEL_SCALE;
    float gx_dps = gx / GYRO_SCALE;
    float gy_dps = gy / GYRO_SCALE;

    roll = atan2(-ay_g, az_g) * rad_to_deg;
    pitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * rad_to_deg;

    roll += gx_dps * deltaTime;
    pitch += gy_dps * deltaTime;
}

char determineOutput(float roll, float pitch) {
    if (131 > roll && roll > 90 && pitch > -11 && pitch < 15) {
        return 'F';
    } else if (-90 > roll && roll > -121 && pitch > -15 && pitch < 10) {
        return 'B';
    } else if (roll > -195 && roll < -140 && pitch > -90 && pitch < -50) {
        return 'L';
    } else if (roll > -135 && roll < -70 && pitch > 65 && pitch < 100) {
        return 'R';
    } else if (roll > 110 && roll < 150 && pitch > -100 && pitch < -35) {
        return 'J';
    } else if (roll > 100 && roll < 125 && pitch > 30 && pitch < 110) {
        return 'I';
    } else if (roll > -125 && roll < -90 && pitch > 30 && pitch < 80) {
        return 'K';
    } else if (roll > -110 && roll < -65 && pitch > -75 && pitch < -35) {
        return 'M';
    } else {
        return 'S';
    }
}
```

## Car Receiver

The `car_receiver` code receives commands from the gesture transmitter and controls the car's motors accordingly. The car is equipped with an ultrasonic sensor to detect obstacles and avoid collisions by stopping the car if an obstacle is too close.

### Code

```cpp
#include <AFMotor.h>
#include <NewPing.h>

#define TRIG_PIN A0 
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define OBSTACLE_PIN 2

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_64KHZ);
AF_DCMotor motor4(4, MOTOR34_64KHZ);

const int maxSpeed = 255;
const int reducedSpeed = maxSpeed / 3;
char val;
int distance = 100;

void setup() {
  Serial.begin(9600);
  pinMode(OBSTACLE_PIN, OUTPUT);
  digitalWrite(OBSTACLE_PIN, LOW);
  distance = readPing();
}

void loop() {
  if(Serial.available()) {
    val = Serial.read();

    if(distance <= 25 && val != 'B') {
      val = 'S';
      digitalWrite(OBSTACLE_PIN, HIGH);
    } else {
      val = val;
      digitalWrite(OBSTACLE_PIN, LOW);
    }

    switch(val) {
      case 'F':
        setMotors(maxSpeed, FORWARD, maxSpeed, FORWARD);
        break;

      case 'B':
        setMotors(maxSpeed, BACKWARD, maxSpeed, BACKWARD);
        break;

      case 'L':
        setMotors(maxSpeed, BACKWARD, maxSpeed, FORWARD);
        break;

      case 'R':
        setMotors(maxSpeed, FORWARD, maxSpeed, BACKWARD);
        break;

      case 'I':
        setMotors(reducedSpeed, FORWARD, maxSpeed, FORWARD);
        break;

      case 'J':
        setMotors(maxSpeed, FORWARD, reducedSpeed, FORWARD);
        break;

      case 'K':
        setMotors(reducedSpeed, BACKWARD, maxSpeed, BACKWARD);
        break;

      case 'M':
        setMotors(maxSpeed, BACKWARD, reducedSpeed, BACKWARD);
        break;

      case 'S':
        stopMotors();
        break;

      default:
        // Do nothing if no valid command is received
        break;
    }
  }
  distance = readPing();
}

int readPing() {
  delay(15);
  int cm = sonar.ping_cm();
  if(cm == 0) {
    cm = 250;
  }
  return cm;
}

void setMotors(int leftSpeed, int leftDir, int rightSpeed, int rightDir) {
  motor1.setSpeed(leftSpeed);
  motor1.run(leftDir);
  motor2.setSpeed(leftSpeed);
  motor2.run(leftDir);
  motor3.setSpeed(rightSpeed);
  motor3.run(rightDir);
  motor4.setSpeed(rightSpeed);
  motor4.run(rightDir);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
```

### Features

- **Obstacle Detection:** The car stops when an obstacle is detected within 25 cm, unless it's moving backward.
- **Motor Control:** The car's motors are controlled based on the commands received, with different speeds for different gestures.

## Hardware Requirements

- MPU6050 accelerometer and gyroscope sensor
- Arduino board (e.g., Uno)
- Bluetooth module (e.g., HC-05)
- Motors and motor driver (e.g., L298N or Adafruit Motor Shield)
- Ultrasonic sensor (e.g., HC-SR04)
- Power supply (e.g., battery pack)
- Connecting wires and breadboard
- Chassis for the car (with wheels and motors)

## Installation

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/yourusername/semi-automatic-vehicle.git
   cd semi-automatic-vehicle
   ```

2. **Install Required Libraries:**
   Ensure you have the following libraries installed in your Arduino IDE:
   - [AFMotor Library](https://github.com/adafruit/Adafruit-Motor-Shield-library)
   - [MPU6050 Library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
   - [NewPing Library](https://bitbucket.org/teckel12/arduino-new-ping/downloads/)
   - [SoftwareSerial Library](https://www.arduino.cc/en/Reference/SoftwareSerial)

3. **Upload the Code:**
   - Open the `gesture_transmitter.ino` file in the Arduino IDE, select the correct board and port, and upload the code to the transmitter device (Arduino with MPU6050 and Bluetooth module).
   - Open the `car_receiver.ino` file, select the correct board and port, and upload the code to the receiver device (Arduino on the vehicle).

## Usage

1. **Set Up the Hardware:**
   - Assemble the vehicle chassis with motors, motor driver, and ultrasonic sensor.
   - Connect the MPU6050 sensor and Bluetooth module to the Arduino for the gesture transmitter.
   - Connect the motor driver, ultrasonic sensor, and Bluetooth module to the Arduino on the vehicle.

2. **Power On:**
   - Power on the Arduino boards and ensure the Bluetooth modules are paired.

3. **Control the Vehicle:**
   - Use specific hand gestures to control the vehicle:
     - **Forward (F):** Move the vehicle forward.
     - **Backward (B):** Move the vehicle backward.
     - **Left (L):** Turn the vehicle left.
     - **Right (R):** Turn the vehicle right.
     - **Stop (S):** Stop the vehicle.
   - The vehicle will automatically stop if an obstacle is detected within 25 cm while moving forward.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

This README provides an overview and usage instructions for the semi-automatic vehicle project. Make sure to customize any URLs or specific information related to your setup.
