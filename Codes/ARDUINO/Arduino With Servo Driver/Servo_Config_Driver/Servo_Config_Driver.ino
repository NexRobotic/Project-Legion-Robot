#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define PCA9685 servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo limits
#define SERVOMIN  150 // Minimum pulse length count
#define SERVOMAX  600 // Maximum pulse length count

// Define servo channels for the PCA9685
const int servo_channels[4][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14} };

// Function to map angle to PCA9685 pulse width
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  // Initialize the PCA9685 driver
  pwm.begin();
  pwm.setPWMFreq(60); // Set frequency to 50 Hz (typical for servos)

  // Initialize all servos to 90 degrees
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      int pulse = angleToPulse(90); // 90 degrees
      pwm.setPWM(servo_channels[i][j], 0, pulse);
      delay(20); // Small delay between servo updates
    }
  }
}

void loop() {
  // Continuously set all servos to 90 degrees
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      int pulse = angleToPulse(90); // 90 degrees
      pwm.setPWM(servo_channels[i][j], 0, pulse);
      delay(20); // Small delay between servo updates
    }
  }
}
