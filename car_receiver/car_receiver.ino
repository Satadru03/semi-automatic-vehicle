//Receiver
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


void setup()
{
  Serial.begin(9600);
  pinMode(OBSTACLE_PIN, OUTPUT);
  digitalWrite(OBSTACLE_PIN, LOW);
  distance = readPing();
}

void loop()
{
  if(Serial.available())
  {
    val = Serial.read();

    if(distance<=25 && val!= 'B')
    {
      val = 'S';
      digitalWrite(OBSTACLE_PIN, HIGH);
    }
    else
    {
      val=val;
      digitalWrite(OBSTACLE_PIN, LOW);
    }

    switch(val)
    {
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

int readPing()
{ 
  delay(15);
  int cm = sonar.ping_cm();
  if(cm==0)
  {
    cm = 250;
  }
  return cm;
}

void setMotors(int leftSpeed, int leftDir, int rightSpeed, int rightDir)
{
  motor1.setSpeed(leftSpeed);
  motor1.run(leftDir);
  motor2.setSpeed(leftSpeed);
  motor2.run(leftDir);
  motor3.setSpeed(rightSpeed);
  motor3.run(rightDir);
  motor4.setSpeed(rightSpeed);
  motor4.run(rightDir);
}

void stopMotors()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}