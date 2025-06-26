/*
** PROJECT: TEST 2 - THE BLACK BOX
** TEAM: EPIBOT
** file desc: Code for The Cube
*/

// ---------- REQUIRED LIBRARIES ----------
#include <Wire.h>              // Enables I2C communication
#include <LiquidCrystal.h> // Library for LCD screen

const int slave_address = 0x20; // slave address on I2C bus

LiquidCrystal lcd(18, 16, 23, 24, 25, -1);

volatile int16_t ax, ay, az;
volatile uint8_t buffer[6];
volatile uint8_t index = 0;

void setup() {
  Wire.begin(slave_address);
  Wire.onReceive(receiveEvent);
  lcd.begin(16, 2);         // Initialize LCD
  lcd.setCursor(0, 0);
  lcd.print("EPIBOT.");
  delay(1000); // wait a second
}

void loop() {
  lcd.setCursor(0, 0); // set cursor to 1st row, 1st column
  // print data on the screen
  lcd.print("X:");
  lcd.print(ax);
  lcd.print(" Y:");
  lcd.print(ay);
  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.print(az);
  delay(500); // print data every half second
}

void receiveEvent(int howMany) {
  index = 0;

  while (Wire.available() && index < 6)
  {
    buffer[index++] = Wire.read();
  }
  if (index == 6) {
    ax = (buffer[0] << 8) | buffer[1];
    ay = (buffer[2] << 8) | buffer[3];
    az = (buffer[4] << 8) | buffer[5];
  }
}
