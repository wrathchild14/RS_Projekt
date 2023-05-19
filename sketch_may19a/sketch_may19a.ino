#include<Wire.h>
#include<Ticker.h>


// <-------------------- I2C  funkciji zacetek -------------------------->
void I2CWriteRegister(uint8_t I2CDevice, uint8_t RegAdress, uint8_t Value){
  // I2CDevice - Naslov I2C naprave
  // RegAddress - Naslov registra 
  // Value - Vrednost za vpisati v register
  
  Wire.beginTransmission(I2CDevice);
  // Napravi sporočimo naslov registra, s katerega želimo brati:
  Wire.write(RegAdress);
  // Posljemo vrednost
  Wire.write(Value);
  Wire.endTransmission();
}

void I2CReadRegister(uint8_t I2CDevice, uint8_t RegAdress, uint8_t NBytes, uint8_t *Value){
  Wire.beginTransmission(I2CDevice);
  // Napravi sporočimo naslov registra, s katerega želimo brati:
  Wire.write(RegAdress);
  // Končamo prenos:
  Wire.endTransmission();
  
  // Napravi sporočimo, da želimo prebrati določeno število 8-bitnih registrov:
  Wire.requestFrom(I2CDevice, NBytes);
  for (int q = 0; q < NBytes; q++) {
    // Preberemo naslednji 8-bitni register oz. naslednji bajt:
    *Value = (uint8_t)Wire.read();
    Value++;
    //uint32_t vrednost = Wire.read();
  }

}

// <-------------------- I2C  Funkcije konec -------------------------->


// Naslov MPU9250 na I2C vodilu
#define MPU_ADD 104
// Naslov registra za pospesek 
#define ACC_MEAS_REG 59
// Naslov registra za ziroskop
#define GYRO_MEAS_REG 67


// Stevilo uzorcev za kalibracijo
#define CAL_NO 1000
// Stevilo uzorcev za branje
#define READ_NO 10
// Globalni stevec zanke 
uint8_t iter = 0;

// gravity
g = 9.81;

// Meritve sa senzorja gyrosa
uint8_t gyroMeas[] = {0,0,0,0,0,0};

// Meritve sa senzorja pospeskometra
uint8_t accelMeas[] = {0,0,0,0,0,0};


Ticker readSensor;
// Kalibrirane vrednosti Accel
float accelX_off = 0; 
float accelY_off = 0; 
float accelZ_off = 0;

// Kalibrirane vrednosti Gyros
float gyroX_off = 0; 
float gyroY_off = 0; 
float gyroZ_off = 0; 
// Meritve 
float gyroX ,gyroY, gyroZ;

void MPU9250_init(){
  // Resetiraj MPU9250 senzora => Register PWR_MGMT_1 (107)
  I2CWriteRegister(MPU_ADD,107,128); // 128 = 1000 0000
  // Pocakaj
  delay(500);
  // Preveri ID od senzora => Register WHO_AM_I (117) 
  uint8_t ID;
  I2CReadRegister(MPU_ADD,117,1,&ID);
  Serial.println("ID:");
  Serial.println(ID, HEX);
  // Gyroscope Conf => Register GYRO_CONFIG (27) 
  // 4 in 3 bit dolocata obseg 
  I2CWriteRegister(MPU_ADD,27,0); // 
  delay(100);
  // Accelerator Conf => Register ACCEL_CONFIG (28)
  // 4 in 3 bit dolocata obseg 
  // Opciono => Register ACCEL_CONFIG_2 (29)
  I2CWriteRegister(MPU_ADD,28,0); // 
  delay(100);
}

void calibrateAccelotemer(){
  uint32_t ITER = 1000; // Stevilo uzorcev za glajenje 
  int32_t tmp;
  Serial.println("Kalibracija Accelometra");
  for(int i=0; i< CAL_NO; i++){
     I2CReadRegister(MPU_ADD,ACC_MEAS_REG,6,accelMeas);
     // ACC_XOUT = Accel_Sensitivity * X_angular_rate
     tmp = (((int8_t)accelMeans[0] << 8) + (uint8_t)accelMeans[1]);
     accelX_off += tmp*1.0/131.0;
      // ACC_YOUT = Accel_Sensitivity * Y_angular_rate
     tmp = (((int8_t)accelMeans[2] << 8) + (uint8_t)accelMeans[3]);
     accelY_off += tmp*1.0/131.0;
     // ACC_ZOUT = Accel_Sensitivity * Z_angular_rate
     tmp = (((int8_t)accelMeans[4] << 8) + (uint8_t)accelMeans[5]);
     accelZ_off += tmp*1.0/131.0;
     Serial.print(".");
  }
  Serial.println("Konec kalibracije");

  gyroX_off /= CAL_NO;
  gyroY_off /= CAL_NO;
  gyroZ_off /= CAL_NO;
  
  Serial.println("Accelometer X osi");
  Serial.println(gyroX_off);
  Serial.println("Accelometer Y osi");
  Serial.println(gyroY_off);
  Serial.println("Accelometer Z osi");
  Serial.println(gyroZ_off);
}

void calibrateGyro(){
  
  uint32_t ITER = 1000; // Stevilo uzorcev za glajenje 
  int32_t tmp;
  Serial.println("Kalibracija ziroskopa");
  for(int i=0; i< CAL_NO; i++){
     I2CReadRegister(MPU_ADD,GYRO_MEAS_REG,6,gyroMeas);
     // GYRO_XOUT = Gyro_Sensitivity * X_angular_rate
     tmp = (((int8_t)gyroMeas[0] << 8) + (uint8_t)gyroMeas[1]);
     gyroX_off += tmp*1.0/131.0;
      // GYRO_YOUT = Gyro_Sensitivity * Y_angular_rate
     tmp = (((int8_t)gyroMeas[2] << 8) + (uint8_t)gyroMeas[3]);
     gyroY_off += tmp*1.0/131.0;
     // GYRO_ZOUT = Gyro_Sensitivity * Z_angular_rate
     tmp = (((int8_t)gyroMeas[4] << 8) + (uint8_t)gyroMeas[5]);
     gyroZ_off += tmp*1.0/131.0;
     Serial.print(".");
  }
  Serial.println("Konec kalibracije");

  gyroX_off /= CAL_NO;
  gyroY_off /= CAL_NO;
  gyroZ_off /= CAL_NO;
  
  Serial.println("Ziroskop X osi");
  Serial.println(gyroX_off);
  Serial.println("Ziroskop Y osi");
  Serial.println(gyroY_off);
  Serial.println("Ziroskop Z osi");
  Serial.println(gyroZ_off);

}

void readAccel(){
  int32_t tmp;
  // TODO
}

void readGyro(){
  int32_t tmp;
  // TODO
}

void setup() {
  // Serijska komunikacija
  Serial.begin(115200);
  // I2C
  Wire.begin(12,14);
  // SDA - 12 pin
  // SCL - 14 pin
  Wire.setClock(100000);
  // Podesavanje senzorja 
  // https://github.com/bolderflight/mpu9250/blob/main/src/mpu9250.cpp
  MPU9250_init();
  // Kalibracija 
  calibrateGyro();
  // Branje senzorja
  readSensor.attach(0.1, readGyro);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}