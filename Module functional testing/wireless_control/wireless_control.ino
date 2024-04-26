#include <Arduino.h>
#define BAUD_RATE 115200 // Define the baud rate

class MotorController {
private:
  int freq_PWM; // Frequency of PWM signal
  int resolution_PWM; // Resolution of PWM signal
  int channel_PWMA, channel_PWMB; // PWM channel numbers
  int PWMA_Pin, PWMB_Pin; // PWM output pin numbers
  int AIN1_Pin, AIN2_Pin, BIN1_Pin, BIN2_Pin; // Motor direction control pins

public:
  // Constructor
  MotorController(int freq, int resolution, int chA, int chB, int pinA, int pinB, int ain1, int ain2, int bin1, int bin2) 
  : freq_PWM(freq), resolution_PWM(resolution), channel_PWMA(chA), channel_PWMB(chB), 
    PWMA_Pin(pinA), PWMB_Pin(pinB), AIN1_Pin(ain1), AIN2_Pin(ain2), BIN1_Pin(bin1), BIN2_Pin(bin2) {
    // Configure PWM channel frequency and resolution
    ledcSetup(channel_PWMA, freq_PWM, resolution_PWM);
    ledcSetup(channel_PWMB, freq_PWM, resolution_PWM);
    // Attach PWM channels to the specified pins
    ledcAttachPin(PWMA_Pin, channel_PWMA);
    ledcAttachPin(PWMB_Pin, channel_PWMB);
    // Set motor direction control pins to output mode
    pinMode(AIN1_Pin, OUTPUT);
    pinMode(AIN2_Pin, OUTPUT);
    pinMode(BIN1_Pin, OUTPUT);
    pinMode(BIN2_Pin, OUTPUT);
  }

  // Set speed for motor A and B (duty cycle), input is a percentage from 0 to 1
  void setSpeed(float speedA, float speedB) {
    int dutyCycleA = speedA * (pow(2, resolution_PWM) - 1);
    int dutyCycleB = speedB * (pow(2, resolution_PWM) - 1);
    ledcWrite(channel_PWMA, dutyCycleA);
    ledcWrite(channel_PWMB, dutyCycleB);
  }

  // Move motors forward
  void forward(float speed) {
    digitalWrite(AIN1_Pin, LOW);
    digitalWrite(AIN2_Pin, HIGH);
    digitalWrite(BIN1_Pin, HIGH);
    digitalWrite(BIN2_Pin, LOW);
    setSpeed(speed, speed);
  }

  // Move motors backward
  void backward(float speed) {
    digitalWrite(AIN1_Pin, HIGH);
    digitalWrite(AIN2_Pin, LOW);
    digitalWrite(BIN1_Pin, LOW);
    digitalWrite(BIN2_Pin, HIGH);
    setSpeed(speed, speed);
  }

  // Turn motors left
  void turnLeft(float speed) {
    digitalWrite(AIN1_Pin, HIGH);
    digitalWrite(AIN2_Pin, LOW);
    digitalWrite(BIN1_Pin, HIGH);
    digitalWrite(BIN2_Pin, LOW);
    setSpeed(speed, speed);
  }

  // Turn motors right
  void turnRight(float speed) {
    digitalWrite(AIN1_Pin, LOW);
    digitalWrite(AIN2_Pin, HIGH);
    digitalWrite(BIN1_Pin, LOW);
    digitalWrite(BIN2_Pin, HIGH);
    setSpeed(speed, speed);
  }
};

// Define PWM signal parameters, channel numbers, and pin numbers
int freq_PWM = 2000;
int resolution_PWM = 8;
MotorController motor(freq_PWM, resolution_PWM, 0, 1, 4, 2, 18, 15, 14, 13);

void setup() {
  motor.setSpeed(0, 0); // Stop the motors
  Serial1.begin(BAUD_RATE, SERIAL_8N1, 40, 1);
  while (!Serial1) {
    ; // Wait for the serial connection. This step is necessary for some boards.
  }
}

void loop() {
  String receivedString = Serial1.readStringUntil('\n'); // Read serial data until a newline character
  if (receivedString == "FORWARD") {
    motor.forward(1);  // Move forward at 100% speed
  }
  else if (receivedString == "BACKWARD") {
    motor.backward(1); // Move backward at 100% speed
  }
  else if (receivedString == "LEFT") {
    motor.turnLeft(0.3); // Turn left at 30% speed
  }
  else if (receivedString == "RIGHT") {
    motor.turnRight(0.3);// Turn right at 30% speed
  }
  else {
    motor.setSpeed(0, 0); // Stop the motors
  }
}
