#include <Wire.h>               // For I2C communication
#include "SSD1306Wire.h"        // OLED display driver library for SSD1306
#include "SR04.h"               // Ultrasonic distance measurement library
#include <Arduino.h>
#define BAUD_RATE 115200 // Define the baud rate

// The I2C address for the OLED display is usually 0x3C or 0x3D, set SDA and SCL pins according to your Arduino model
SSD1306Wire display(0x3c, 21, 47);  // Modify SDA and SCL pin numbers based on actual situation

#define TRIG_PIN 10
#define ECHO_PIN 6
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
double distance; 

const uint16_t AENCA = 36;  // Encoder A input A_C2(B)
const uint16_t AENCB = 3;   // Encoder A input A_C1(A)
const uint16_t BENCB = 35;  // Encoder B input B_C2(B)
const uint16_t BENCA = 39;  // Encoder B input B_C1(A)

volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count = 0;

void IRAM_ATTR B_wheel_pulse() {
  if (digitalRead(BENCA) == HIGH) {
    B_wheel_pulse_count++;
  } else {
    B_wheel_pulse_count--;
  }
}

void IRAM_ATTR A_wheel_pulse() {
  if (digitalRead(AENCA) == HIGH) {
    A_wheel_pulse_count++;
  } else {
    A_wheel_pulse_count--;
  }
}

String createDataFrame(double distance, long pulseA, long pulseB) {
  String frame;
  // Package data into a simple comma-separated string
  frame += String(distance) + ",";
  frame += String(pulseA) + ",";
  frame += String(pulseB);
  return frame;
}

void setup() {
    // Initialize serial communication RXPIN, TXPIN
  Serial1.begin(BAUD_RATE, SERIAL_8N1, 40, 1);
  while (!Serial1) {
    ; // Wait for the serial connection. This step is necessary for some boards.
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
  String receivedString = Serial1.readStringUntil('\n');
  display.drawString(0, 45, receivedString);
  display.display();

  // Create and send a data frame
  String frame = createDataFrame(distance, currentACount, currentBCount);
  Serial1.println(frame);
}
