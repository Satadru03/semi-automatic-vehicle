#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

MPU6050 mpu;

SoftwareSerial bluetoothSerial(0, 1);

float roll, pitch;
const float rad_to_deg = 57.2958;
char out;

// Sensitivity scales for calibration
const float ACCEL_SCALE = 8192.0;  // Scale factor for accelerometer at ±4g (8192 LSB/g)
const float GYRO_SCALE = 65.5;     // Scale factor for gyroscope at ±500 dps (65.5 LSB/dps)

unsigned long prevTime;
float deltaTime;

void setup()
{
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

void loop()
{
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate deltaTime in seconds
  unsigned long currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Calculate roll and pitch
  calculateAngles(ax, ay, az, gx, gy);

  // Determine the output command based on roll and pitch
  out = determineOutput(roll, pitch);

  Serial.println(out);
  bluetoothSerial.println(out);

  delay(200);
}

void calculateAngles(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy)
{
  // Convert accelerometer values to g's
  float ax_g = ax / ACCEL_SCALE;
  float ay_g = ay / ACCEL_SCALE;
  float az_g = az / ACCEL_SCALE;

  // Convert gyroscope values to degrees per second
  float gx_dps = gx / GYRO_SCALE;
  float gy_dps = gy / GYRO_SCALE;

  // Calculate roll and pitch
  roll = atan2(-ay_g, az_g) * rad_to_deg;
  pitch = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * rad_to_deg;

  // Update roll and pitch based on gyroscope readings and deltaTime
  roll += gx_dps * deltaTime;
  pitch += gy_dps * deltaTime;
}

char determineOutput(float roll, float pitch)
{
  if (131 > roll && roll > 90 && pitch > -11 && pitch < 15)
  {
    return 'F';
  }
  else if (-90 > roll && roll > -121 && pitch > -15 && pitch < 10)
  {
    return 'B';
  }
  else if (roll > -195 && roll < -140 && pitch > -90 && pitch < -50)
  {
    return 'L';
  }
  else if (roll > -135 && roll < -70 && pitch > 65 && pitch < 100)
  {
    return 'R';
  }
  else if (roll > 110 && roll < 150 && pitch > -100 && pitch < -35)
  {
    return 'J';
  }
  else if (roll > 100 && roll < 125 && pitch > 30 && pitch < 110)
  {
    return 'I';
  }
  else if (roll > -125 && roll < -90 && pitch > 30 && pitch < 80)
  {
    return 'K';
  }
  else if (roll > -110 && roll < -65 && pitch > -75 && pitch < -35)
  {
    return 'M';
  }
  else
  {
    return 'S';
  }
}