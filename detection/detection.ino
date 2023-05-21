#include <Wire.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// registers
#define MAIN_REG_ADDR 0x68
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

const bool DEBUG_MODE = true;
const bool COMPOMENTARY_FILTER = false;

// calibration values
float accel_off_x = 0.0f;
float accel_off_y = 0.0f;
float accel_off_z = 0.0f;
float gyro_off_x = 0.0f;
float gyro_off_y = 0.0f;
float gyro_off_z = 0.0f;

if (COMPOMENTARY_FILTER) {
  const float WALKING_THRESHOLD_VER = 0.2f;
  const float WALKING_THRESHOLD_HOR = 0.2f;
  const float JUMPING_THRESHOLD_VER = 0.8f;
  const float RUNNING_THRESHOLD_VER = 0.4f;
  const float RUNNING_THRESHOLD_HOR = 0.4f;
  const float BIKING_THRESHOLD_VER = 0.2f;
  const float BIKING_THRESHOLD_HOR = 0.4f;
} else {
  // old values without complementary filter
  const float WALKING_THRESHOLD_VER = 0.1f;
  const float WALKING_THRESHOLD_HOR = 0.05f;
  const float JUMPING_THRESHOLD_VER = 0.7f;
  const float RUNNING_THRESHOLD_VER = 0.5f;
  const float RUNNING_THRESHOLD_HOR = 0.15f;
  const float BIKING_THRESHOLD_VER = 0.2f;
  const float BIKING_THRESHOLD_HOR = 0.2f;
}

const int TICKER_INTERVAL = 100;
const float ALPHA = 0.98f;
const float DT = 0.1f;

Ticker ticker;

// classification properties
const int DATA_SIZE = 10;
float vertical_data[DATA_SIZE];
float horizontal_data[DATA_SIZE];
int data_idx = 0;
float get_std_dev(const float* data, int data_size);

const char* ssid = "Oogabooga";
const char* password = "monkiestuff";

ESP8266WebServer server(80);
String activity = "unknown";

void handleRoot() {
  server.send(200, "text/plain", activity);
}

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

  float roll = atan2(acc_y, acc_z);
  float pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));

  if (COMPLEMENTARY_FILTER) {
    roll = ALPHA * (roll + gyro_x * DT) + (1.0f - ALPHA) * roll;
    pitch = ALPHA * (pitch + gyro_y * DT) + (1.0f - ALPHA) * pitch;
  }

  float vert_comp_acc = acc_z * cos(pitch);
  float forward = acc_x * cos(pitch) + acc_y * sin(pitch) * sin(roll);
  float right = acc_y * cos(roll) - acc_z * sin(roll) * sin(pitch);
  float hor_comp_acc = sqrt(forward * forward + right * right);

  vertical_data[data_idx] = vert_comp_acc;
  horizontal_data[data_idx] = hor_comp_acc;
  data_idx = (data_idx + 1) % DATA_SIZE;

  float vert_std_dev = get_std_dev(vertical_data, DATA_SIZE);
  float hor_std_dev = get_std_dev(horizontal_data, DATA_SIZE);

  if (DEBUG_MODE) {
    float roll_deg = roll * (180.0 / PI);
    float pitch_deg = pitch * (180.0 / PI);
    Serial.print("roll_deg: ");
    Serial.print(roll_deg);
    Serial.print(" pitch_deg: ");
    Serial.print(pitch_deg);

    Serial.print(", forward: ");
    Serial.print(forward);
    Serial.print(" right: ");
    Serial.print(right);

    Serial.print(", hor_comp_acc: ");
    Serial.print(hor_comp_acc);

    Serial.print(" vert_std_dev: ");
    Serial.print(vert_std_dev);
    Serial.print(" hor_std_dev: ");
    Serial.print(hor_std_dev);
    Serial.println();
  }

  String new_activity;
  if (vert_std_dev > WALKING_THRESHOLD_VER && hor_std_dev > WALKING_THRESHOLD_HOR && vert_std_dev < RUNNING_THRESHOLD_VER && hor_std_dev < RUNNING_THRESHOLD_HOR) {
    new_activity = "walking";
  } else if (vert_std_dev > RUNNING_THRESHOLD_VER && hor_std_dev > RUNNING_THRESHOLD_HOR && vert_std_dev < JUMPING_THRESHOLD_VER) {
    new_activity = "running";
  } else if (vert_std_dev > JUMPING_THRESHOLD_VER) {
    new_activity = "jumping";
  } else if (vert_std_dev > BIKING_THRESHOLD_VER && hor_std_dev > BIKING_THRESHOLD_HOR && vert_std_dev < RUNNING_THRESHOLD_VER) {
    new_activity = "riding";
  } else {
    new_activity = "standing";
  }

  if (activity != new_activity) {
    activity = new_activity;
    server.send(200, "text/plain", new_activity);

    Serial.println(new_activity);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(12, 14);

  Serial.println();
  Serial.println("trying to connect to server");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("wifi connected, ur local ip: ");
  Serial.print(WiFi.localIP());
  Serial.println();

  server.on("/", handleRoot);
  server.begin();
  Serial.println("http server started");


  calibrate();
  ticker.attach_ms(TICKER_INTERVAL, activity_detect);
}

void loop() {
  server.handleClient();
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
