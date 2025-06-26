/*
** PROJECT: TEST 2 - THE BLACK BOX
** TEAM: EPIBOT
** file desc: Code for The Cube
*/

// ---------- REQUIRED LIBRARIES ----------
#include <Wire.h>              // Enables I2C communication
#include <MPU6050.h>           // Library for MPU6050 sensor

MPU6050 mpu; // represents the sensor
const int slave_address = 0x20; // slave address on I2C bus

void setup()
{
  Wire.begin(); // Join I2C as Master
  Serial.begin(9600);
  mpu.initialize();
}

void loop()
{
  
  int16_t aX, aY, aZ; // store sensor data
  mpu.getAcceleration(&aX, &aY, &aZ);

// =========== send data Control Station (slave) ============
  Wire.beginTransmission(slave_address);
  Wire.write((byte)(aX >> 8));
  Wire.write((byte)aX);
  Wire.write((byte)(aY >> 8));
  Wire.write((byte)aY);
  Wire.write((byte)(aZ >> 8));
  Wire.write((byte)aZ);
  Wire.endTransmission();
  delay(500); // Send data every half second
}
