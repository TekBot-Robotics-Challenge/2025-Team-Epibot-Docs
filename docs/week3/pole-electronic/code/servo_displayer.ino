/*
** PROJECT: TEST 3 - AFFICHEUR 7 SEGMENTS 
** TEAM: EPIBOT
*/

/* ========= REQUIRED LIBRAIRIES ============== */
#include <Wire.h>                     // Enables I2C communication
#include <Adafruit_PWMServoDriver.h>  // Control the PCA-9685

// create a PWM controller for the PCA-9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define pulseMin 125 // 0 degree pulse
#define pulseMax 625 // 180 degrees pulse

#define numberOfServos 7

unsigned long previousMillis = 0;
const long interval = 1000; // Interval of 1 second

// boolean-array for active/non-active states of the servos
// when displaying each digit
const bool digitPositions[10][7] =
{
  {1, 1, 1, 1, 1, 1, 0}, // digit 0
  {0, 1, 1, 0, 0, 0, 0}, // digit 1
  {1, 1, 0, 1, 1, 0, 1}, // digit 2
  {1, 1, 1, 1, 0, 0, 1}, // digit 3
  {0, 1, 1, 0, 0, 1, 1}, // digit 4
  {1, 0, 1, 1, 0, 1, 1}, // digit 5
  {1, 0, 1, 1, 1, 1, 1}, // digit 6
  {1, 1, 1, 0, 0, 0, 0}, // digit 7
  {1, 1, 1, 1, 1, 1, 1}, // digit 8
  {1, 1, 1, 1, 0, 1, 1}, // digit 9
};

#define UP 90 // segment up position angle (active state)
#define FLAT 180 // segment flat position angle (non-active state)

// variable to track current digit
unsigned int currentDigit = 0;

void setup()
{
  Serial.begin(115200); // Start the serial monitor at 115200 bauds
  pwm.begin(); // Start the PWM controller
  pwm.setPWMFreq(60); // Set servo frequency at 60 Hz
  // put all servo in non-active mode at the start
  const bool startPos[7] = {0, 0, 0, 0, 0, 0, 0};
  displayDigit(startPos);
  Serial.println("Successful setup !"); // for debugging purposes
}

void loop()
{
  unsigned long currentMillis = millis(); // Get elapsed time
  // control digit order: if false, go from 0 to 9, else, go from 9 to 0
  static bool rewind = false; // display

  // ensure the time interval is respected...
  // action is done every interval seconds
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Save current time
    // print to the serial monitor for debugging
    Serial.print("==="); Serial.print(currentDigit); Serial.println("===");
    // display the current digit
    displayDigit(digitPositions[currentDigit]);

    if (currentDigit >= 9)
      rewind = true; // start counting backwards
    if (!rewind)
      currentDigit++;
    if (rewind && currentDigit > 0)
      currentDigit--;
    if (rewind && !currentDigit)
      rewind = false; // start counting from zero again
  }
}

void displayDigit(const bool array[7])
{
  for (int i = 0; i < numberOfServos; i++)
  {
    if (array[i])
    {
      // print the serial monitor for debugging
      Serial.print("servo "); Serial.print(i); Serial.println(" is on");
      moveServo(i, UP); // move segment up
    }
    else
    {
      // print the serial monitor for debugging
      Serial.print("servo "); Serial.print(i); Serial.println(" is off");
      moveServo(i, FLAT); // lay segment flat
    }
  }
}

void moveServo(int servoNumber, int angle)
{
  int pulse = map(angle, UP, FLAT, pulseMin, pulseMax); // convert angle to pulse
  pwm.setPWM(servoNumber, 0, pulse); // apply pulse to servo
}
