#include <Wire.h>
#include <Ticker.h>

// registers
#define MAIN_REG_ADDR 0x68
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

// calibration values
float accelOffsetX = 0.0f;
float accelOffsetY = 0.0f;
float accelOffsetZ = 0.0f;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

const float ACCELERATION_THRESHOLD = 0.8f;
const float PEAK_MINIMUM_THRESHOLD = 0.5f;
const float JUMP_THRESHOLD = 1.5f;

const int TICKER_INTERVAL = 100;

Ticker ticker;

float previousAccelValue = 0.0f;
float currentAccelValue = 0.0f;
bool isStep = false;
bool isJump = false;

void calibrateSensors() {
  int iterations = 500;

  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;

  for (int i = 0; i < iterations; i++) {
    Wire.beginTransmission(MAIN_REG_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MAIN_REG_ADDR, 12, true);

    int16_t accelRawX = (Wire.read() << 8) | Wire.read();
    int16_t accelRawY = (Wire.read() << 8) | Wire.read();
    int16_t accelRawZ = (Wire.read() << 8) | Wire.read();

    int16_t gyroRawX = (Wire.read() << 8) | Wire.read();
    int16_t gyroRawY = (Wire.read() << 8) | Wire.read();
    int16_t gyroRawZ = (Wire.read() << 8) | Wire.read();

    accelX += accelRawX / 16384.0f;
    accelY += accelRawY / 16384.0f;
    accelZ += accelRawZ / 16384.0f;
    gyroX += gyroRawX / 131.0f;
    gyroY += gyroRawY / 131.0f;
    gyroZ += gyroRawZ / 131.0f;

    delay(2);
  }

  // [-1, 1]
  accelOffsetX = accelX / iterations;
  accelOffsetY = accelY / iterations;
  // [0, 2]
  accelOffsetZ = (accelZ / iterations) - 1.0f;  // 1g for gravity

  gyroOffsetX = gyroX / iterations;
  gyroOffsetY = gyroY / iterations;
  gyroOffsetZ = gyroZ / iterations;

  Serial.println("Sensor calibration complete.");
  Serial.print("Accelerometer offsets (X,Y,Z): ");
  Serial.print(accelOffsetX);
  Serial.print(", ");
  Serial.print(accelOffsetY);
  Serial.print(", ");
  Serial.println(accelOffsetZ);
  Serial.print("Gyroscope offsets (X,Y,Z): ");
  Serial.print(gyroOffsetX);
  Serial.print(", ");
  Serial.print(gyroOffsetY);
  Serial.print(", ");
  Serial.println(gyroOffsetZ);
}

void detectWalking() {
  Wire.beginTransmission(MAIN_REG_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(MAIN_REG_ADDR, 12);

  int16_t accelRawX = (Wire.read() << 8) | Wire.read();
  int16_t accelRawY = (Wire.read() << 8) | Wire.read();
  int16_t accelRawZ = (Wire.read() << 8) | Wire.read();

  int16_t gyroRawX = (Wire.read() << 8) | Wire.read();
  int16_t gyroRawY = (Wire.read() << 8) | Wire.read();
  int16_t gyroRawZ = (Wire.read() << 8) | Wire.read();

  // 2^14
  float accelX = (accelRawX / 16384.0f) - accelOffsetX;
  float accelY = (accelRawY / 16384.0f) - accelOffsetY;
  float accelZ = (accelRawZ / 16384.0f) - accelOffsetZ;
  float gyroX = (gyroRawX / 131.0f) - gyroOffsetX;
  float gyroY = (gyroRawY / 131.0f) - gyroOffsetY;
  float gyroZ = (gyroRawZ / 131.0f) - gyroOffsetZ;

  Serial.print(accelX);
  Serial.print(" ");
  Serial.print(accelY);
  Serial.print(" ");
  Serial.print(accelZ);

  Serial.println();

  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print(gyroZ);

  Serial.println();

  float acceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  float roll = atan2(accelY, accelZ) * (180.0 / PI);
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * (180.0 / PI);

  if (previousAccelValue > currentAccelValue && previousAccelValue > acceleration && previousAccelValue > PEAK_MINIMUM_THRESHOLD) {
    isStep = true;
  } else {
    isStep = false;
  }

  if (acceleration > JUMP_THRESHOLD) {
    isJump = true;
  } else {
    isJump = false;
  }

  previousAccelValue = currentAccelValue;
  currentAccelValue = acceleration;

  Serial.print("acceleration: ");
  Serial.print(acceleration);
  Serial.print("   roll: ");
  Serial.print(roll);
  Serial.print("   pitch: ");
  Serial.print(pitch);

  if (isStep) {
    Serial.println("step");
  }

  if (isJump) {
    Serial.println("jump");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(12, 14);
  calibrateSensors();
  ticker.attach_ms(TICKER_INTERVAL, detectWalking);
}

void loop() {
}
