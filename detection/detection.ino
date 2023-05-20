#include <Wire.h>
#include <Ticker.h>

// registers
#define MAIN_REG_ADDR 0x68
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

// calibration values
float accel_off_x = 0.0f;
float accel_off_y = 0.0f;
float accel_off_z = 0.0f;
float gyro_off_x = 0.0f;
float gyro_off_y = 0.0f;
float gyro_off_z = 0.0f;

const float WALKING_THRESHOLD = 1.f;
const float RUNNING_THRESHOLD = 4.f;

const int TICKER_INTERVAL = 100;

Ticker ticker;

// classification properties
const int DATA_SIZE = 10;
float roll_data[DATA_SIZE];
float pitch_data[DATA_SIZE];
float vertical_data[DATA_SIZE];
float horizontal_data[DATA_SIZE];
int data_idx = 0;
float get_std_dev(const float* data, int data_size);

void calibrate() {
  int iterations = 500;

  float acc_x = 0.0f;
  float acc_y = 0.0f;
  float acc_z = 0.0f;
  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;

  for (int i = 0; i < iterations; i++) {
    Wire.beginTransmission(MAIN_REG_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MAIN_REG_ADDR, 6);

    int16_t acc_raw_x = (Wire.read() << 8) | Wire.read();
    int16_t acc_ray_y = (Wire.read() << 8) | Wire.read();
    int16_t acc_ray_z = (Wire.read() << 8) | Wire.read();

    Wire.beginTransmission(MAIN_REG_ADDR);
    Wire.write(REG_GYRO_XOUT_H);
    Wire.endTransmission();

    Wire.requestFrom(MAIN_REG_ADDR, 6);

    int16_t gyro_raw_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_raw_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_raw_z = (Wire.read() << 8) | Wire.read();

    acc_x += acc_raw_x / 16384.0f;
    acc_y += acc_ray_y / 16384.0f;
    acc_z += acc_ray_z / 16384.0f;
    gyro_x += gyro_raw_x / 131.0f;
    gyro_y += gyro_raw_y / 131.0f;
    gyro_z += gyro_raw_z / 131.0f;

    delay(2);
  }

  // [-1, 1]
  accel_off_x = acc_x / iterations;
  accel_off_y = acc_y / iterations;
  // [0, 2]
  accel_off_z = (acc_z / iterations) - 1.0f;  // 1g for gravity

  gyro_off_x = gyro_x / iterations;
  gyro_off_y = gyro_y / iterations;
  gyro_off_z = gyro_z / iterations;

  Serial.println("calibration complete");
  Serial.print("acc offsets (X, Y, Z): ");
  Serial.print(accel_off_x);
  Serial.print(", ");
  Serial.print(accel_off_y);
  Serial.print(", ");
  Serial.println(accel_off_z);
  Serial.print("gyroscope offsets (X, Y, Z): ");
  Serial.print(gyro_off_x);
  Serial.print(", ");
  Serial.print(gyro_off_y);
  Serial.print(", ");
  Serial.println(gyro_off_z);
}

void activity_detect() {
  Wire.beginTransmission(MAIN_REG_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(MAIN_REG_ADDR, 6);

  int16_t acc_raw_x = (Wire.read() << 8) | Wire.read();
  int16_t acc_ray_y = (Wire.read() << 8) | Wire.read();
  int16_t acc_ray_z = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(MAIN_REG_ADDR);
  Wire.write(REG_GYRO_XOUT_H);
  Wire.endTransmission();

  Wire.requestFrom(MAIN_REG_ADDR, 6);

  int16_t gyro_raw_x = (Wire.read() << 8) | Wire.read();
  int16_t gyro_raw_y = (Wire.read() << 8) | Wire.read();
  int16_t gyro_raw_z = (Wire.read() << 8) | Wire.read();

  // 2^14
  float acc_x = (acc_raw_x / 16384.0f) - accel_off_x;
  float acc_y = (acc_ray_y / 16384.0f) - accel_off_y;
  float acc_z = (acc_ray_z / 16384.0f) - accel_off_z;
  float gyro_x = (gyro_raw_x / 131.0f) - gyro_off_x;
  float gyro_y = (gyro_raw_y / 131.0f) - gyro_off_y;
  float gyro_z = (gyro_raw_z / 131.0f) - gyro_off_z;

  // pritning of accel and gyro data
  // Serial.print(acc_x);
  // Serial.print(" ");
  // Serial.print(acc_y);
  // Serial.print(" ");
  // Serial.print(acc_z);
  // Serial.println();
  // Serial.print(gyro_x);
  // Serial.print(" ");
  // Serial.print(gyro_y);
  // Serial.print(" ");
  // Serial.print(gyro_z);
  // Serial.println();

  float acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

  float roll = atan2(acc_y, acc_z);
  float pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));

  float vert_comp_acc = acc_z * cos(pitch);
  // float hor_comp_acc = acc_z * sin(pitch * (PI / 180.0));
  // float hor_comp_acc = sqrt(acc_x * acc_x + acc_y * acc_y) * sin(pitch * (PI / 180.0));

  // Serial.print("acceleration: ");
  // Serial.print(acceleration);
  // Serial.print("   roll: ");
  // Serial.print(roll);
  // Serial.print("   pitch: ");
  // Serial.print(pitch);
  // Serial.println();

  roll_data[data_idx] = roll;
  pitch_data[data_idx] = pitch;
  vertical_data[data_idx] = vert_comp_acc;
  horizontal_data[data_idx] = hor_comp_acc;
  data_idx = (data_idx + 1) % DATA_SIZE;

  float roll_std_dev = get_std_dev(roll_data, DATA_SIZE);
  float pitch_std_dev = get_std_dev(pitch_data, DATA_SIZE);
  float vert_std_dev = get_std_dev(vertical_data, DATA_SIZE);
  float hor_std_dev = get_std_dev(horizontal_data, DATA_SIZE);

  Serial.print("roll_std_dev: ");
  Serial.print(roll_std_dev);
  Serial.print(" pitch_std_dev: ");
  Serial.print(pitch_std_dev);
  Serial.print(" vert_std_dev: ");
  Serial.print(vert_std_dev);
  Serial.print(" hor_std_dev: ");
  Serial.print(hor_std_dev);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(12, 14);
  calibrate();
  ticker.attach_ms(TICKER_INTERVAL, activity_detect);
}

void loop() {
}

float get_std_dev(const float* data, int data_size) {
  float sum = 0.0f;
  float mean = 0.0f;

  for (int i = 0; i < data_size; i++) {
    sum += data[i];
  }
  mean = sum / data_size;

  float squared_diff_sum = 0.0f;
  for (int i = 0; i < data_size; i++) {
    float diff = data[i] - mean;
    squared_diff_sum += (diff * diff);
  }

  float variance = squared_diff_sum / data_size;
  float std_dev = sqrt(variance);

  return std_dev;
}
