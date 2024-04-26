#include <Wire.h>               // Include Wire library for I2C communication
#include "SSD1306Wire.h"        // Library for SSD1306 OLED display
#include "SR04.h"               // Ultrasonic distance measurement library
#include <Arduino.h>
#define BAUD_RATE 9600          // Define baud rate for serial communication

// OLED display I2C address is typically 0x3C or 0x3D, SDA and SCL pins depend on your Arduino model
SSD1306Wire display(0x3c, 21, 47);  // SDA and SCL pin numbers need to be adjusted according to your setup

#define TRIG_PIN 10
#define ECHO_PIN 6
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);  // Initialize SR04 ultrasonic sensor
double distance; 

const uint16_t AENCA = 36;  // Encoder A input A_C2(B)
const uint16_t AENCB = 3;   // Encoder A input A_C1(A)
const uint16_t BENCB = 35;  // Encoder B input B_C2(B)
const uint16_t BENCA = 39;  // Encoder B input B_C1(A)

volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count = 0;

void IRAM_ATTR B_wheel_pulse() {
  if (digitalRead(BENCA) == HIGH) {
    B_wheel_pulse_count++;   // Increment pulse count
  } else {
    B_wheel_pulse_count--;   // Decrement pulse count
  }
}

void IRAM_ATTR A_wheel_pulse() {
  if (digitalRead(AENCA) == HIGH) {
    A_wheel_pulse_count++;   // Increment pulse count
  } else {
    A_wheel_pulse_count--;   // Decrement pulse count
  }
}

void setup() {
    // Initialize serial communication
  Serial1.begin(BAUD_RATE, SERIAL_8N1, 40, 1);
  while (!Serial1) {
    ; // Wait for serial connection. 
  }
  
  pinMode(BENCB, INPUT_PULLUP);
  pinMode(BENCA, INPUT_PULLUP);
  pinMode(AENCB, INPUT_PULLUP);
  pinMode(AENCA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void loop() {
  distance = sr04.Distance();
  
  noInterrupts();
  long currentBCount = B_wheel_pulse_count;
  long currentACount = A_wheel_pulse_count;
  interrupts();

  display.clear();
  display.drawString(0, 0, "Distance: " + String(distance) + "cm");
  display.drawString(0, 15, "A Pulse: " + String(currentACount));
  display.drawString(0, 30, "B Pulse: " + String(currentBCount));
  String receivedString = Serial1.readStringUntil('\n'); // Read serial data until newline character
  display.drawString(0, 45, receivedString); // Display the received string on OLED
  display.display();
}
