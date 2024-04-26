#include <Arduino.h>
#define BAUD_RATE 115200 // Set communication baud rate
#include <Wire.h>         // Include I2C communication library
#include "SSD1306Wire.h"  // OLED display driver library for SSD1306
#include "SR04.h"         // Ultrasonic distance module library
#define CarID 2

// OLED display's I2C address and the SDA and SCL pin numbers
SSD1306Wire display(0x3c, 21, 47);  

// Pin definitions for the ultrasonic module
#define TRIG_PIN 10
#define ECHO_PIN 6
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);  // Create ultrasonic distance object
double distance; // Variable to store measured distance

// Encoder pin definitions
const uint16_t AENCA = 36;  
const uint16_t AENCB = 3;   
const uint16_t BENCB = 35;  
const uint16_t BENCA = 39;  

// Encoder pulse counts
volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count = 0;

// State enum definition
enum State { START_PAGE, REMOTE_CONTROL, INFO_COLLECTION, DISTANCE_PID };
volatile State currentState = START_PAGE;

// Command information structure definition
struct CommandInfo {
  int carID;
  int mode;
  int start;
  State state;
  String command;
};

// A wheel encoder interrupt service routine
void IRAM_ATTR A_wheel_pulse() {
  static bool lastAState = LOW;
  bool currentAState = digitalRead(AENCA);
  if (currentAState != lastAState) {  // Only count on state change
    if (digitalRead(AENCB) == currentAState) {
      A_wheel_pulse_count++;
    } else {
      A_wheel_pulse_count--;
    }
  }
  lastAState = currentAState;
}

// B wheel encoder interrupt service routine
void IRAM_ATTR B_wheel_pulse() {
  static bool lastBState = LOW;
  bool currentBState = digitalRead(BENCA);
  if (currentBState != lastBState) {  // Only count on state change
    if (digitalRead(BENCB) == currentBState) {
      B_wheel_pulse_count++;
    } else {
      B_wheel_pulse_count--;
    }
  }
  lastBState = currentBState;
}

// Function to generate data frame
String createDataFrame(double distance, long pulseA, long pulseB) {
  String frame;
  frame += String(distance) + ",";
  frame += String(pulseA) + ",";
  frame += String(pulseB);
  return frame;
}

// Motor control class
class MotorController {
private:
  int freq_PWM;  // Frequency of PWM signal
  int resolution_PWM;  // Resolution of PWM signal
  int channel_PWMA, channel_PWMB;  // PWM channel numbers
  int PWMA_Pin, PWMB_Pin;  // PWM output pin numbers
  int AIN1_Pin, AIN2_Pin, BIN1_Pin, BIN2_Pin;  // Motor direction control pins

  double kp = 0.6, ki = 0.2, kd = 0.4;  // PID control parameters
  double setpointA = 0, setpointB = 0;  // PID setpoints
  double integralA = 0, integralB = 0;  // PID integral terms
  double lastErrorA = 0, lastErrorB = 0;  // Last PID error
  unsigned long lastTime = 0;  // Last time record

public:
  // Constructor
  MotorController(int freq, int resolution, int chA, int chB, int pinA, int pinB, int ain1, int ain2, int bin1, int bin2)
  : freq_PWM(freq), resolution_PWM(resolution), channel_PWMA(chA), channel_PWMB(chB), 
    PWMA_Pin(pinA), PWMB_Pin(pinB), AIN1_Pin(ain1), AIN2_Pin(ain2), BIN1_Pin(bin1), BIN2_Pin(bin2) {
    ledcSetup(channel_PWMA, freq_PWM, resolution_PWM);
    ledcSetup(channel_PWMB, freq_PWM, resolution_PWM);
    ledcAttachPin(PWMA_Pin, channel_PWMA);
    ledcAttachPin(PWMB_Pin, channel_PWMB);
    pinMode(AIN1_Pin, OUTPUT);
    pinMode(AIN2_Pin, OUTPUT);
    pinMode(BIN1_Pin, OUTPUT);
    pinMode(BIN2_Pin, OUTPUT);
  }

  // Compute PID and set speed
  void computePID() {
    long encoderA = A_wheel_pulse_count;
    long encoderB = B_wheel_pulse_count;
    unsigned long currentTime = millis();
    double timeChange = (double)(currentTime - lastTime);
    
    if (timeChange >= 100) {
      double errorA = setpointA - encoderA;
      integralA += errorA * timeChange;
      double derivativeA = (errorA - lastErrorA) / timeChange;
      double outputA = kp * errorA + ki * integralA + kd * derivativeA;
      lastErrorA = errorA;

      double errorB = setpointB - encoderB;
      integralB += errorB * timeChange;
      double derivativeB = (errorB - lastErrorB) / timeChange;
      double outputB = kp * errorB + ki * integralB + kd * derivativeB;
      lastErrorB = errorB;

      lastTime = currentTime;

      setSpeed(outputA / (pow(2, resolution_PWM) - 1), outputB / (pow(2, resolution_PWM) - 1));
    }
  }

  // Set motor speeds
  void setSpeed(float speedA, float speedB) {
    int dutyCycleA = speedA * (pow(2, resolution_PWM) - 1);
    int dutyCycleB = speedB * (pow(2, resolution_PWM) - 1);
    ledcWrite(channel_PWMA, constrain(dutyCycleA, 0, 255));
    ledcWrite(channel_PWMB, constrain(dutyCycleB, 0, 255));
  }

  // Set speed setpoints
  void setSpeeds(double speedA, double speedB) {
    setpointA = speedA;
    setpointB = speedB;
  }

  // Control motor to move forward
  void forward(float speed) {
    digitalWrite(AIN1_Pin, LOW);
    digitalWrite(AIN2_Pin, HIGH);
    digitalWrite(BIN1_Pin, HIGH);
    digitalWrite(BIN2_Pin, LOW);
    setSpeeds(speed, speed);
    computePID();
    setSpeed(speed, speed);
  }

  // Control motor to move backward
  void backward(float speed) {
    digitalWrite(AIN1_Pin, HIGH);
    digitalWrite(AIN2_Pin, LOW);
    digitalWrite(BIN1_Pin, LOW);
    digitalWrite(BIN2_Pin, HIGH);
    setSpeeds(speed, speed);
    computePID();
    setSpeed(speed, speed);
  }

  // Control motor to turn left
  void turnLeft(float speed) {
    digitalWrite(AIN1_Pin, HIGH);
    digitalWrite(AIN2_Pin, LOW);
    digitalWrite(BIN1_Pin, HIGH);
    digitalWrite(BIN2_Pin, LOW);
    setSpeed(speed, speed);
  }

  // Control motor to turn right
  void turnRight(float speed) {
    digitalWrite(AIN1_Pin, LOW);
    digitalWrite(AIN2_Pin, HIGH);
    digitalWrite(BIN1_Pin, LOW);
    digitalWrite(BIN2_Pin, HIGH);
    setSpeed(speed, speed);
  }
};

// Initialize motor controller instance
int freq_PWM = 2000; // PWM frequency
int resolution_PWM = 8; // PWM resolution
MotorController motor(freq_PWM, resolution_PWM, 0, 1, 4, 2, 18, 15, 14, 13);

double setPoint = 20;  // Target distance, can be updated via serial port
double pidOutput = 0;  // PID control output
double kp = 0.3, ki = 0, kd = 0.001;  // PID control parameters
double lastError = 0, integral = 0;  // PID error and integral
unsigned long lastTime;  // Last timing

// PID calculation function
void computePID() {
  double currentDistance = sr04.Distance();
  double error = setPoint - currentDistance;  // Calculate distance error
  unsigned long currentTime = millis();
  unsigned long timeChange = currentTime - lastTime;

  if (timeChange >= 100) {  // Update every 100 milliseconds
    // Integral part, with anti-windup handling
    integral += error * timeChange;
    integral = constrain(integral, -500, 500);  // Prevent excessive integral accumulation

    // Derivative part
    double derivative = (error - lastError) / timeChange;

    // Calculate PID output
    pidOutput = kp * error + ki * integral + kd * derivative;
    lastError = error;
    lastTime = currentTime;

    // Adjust motor speed based on PID output, limit output value between -1.0 and 1.0
    float speed = constrain(pidOutput, -0.3, 0.3);
    // Control motor direction based on the speed sign
    if (speed < 0) {
      motor.forward(abs(speed));  // Move forward
    } else {
      motor.backward(abs(speed));  // Move backward
    }
  }
}

int last_move = 0;

// Process received commands
void handleCommand(String cmd) {
    if (cmd == "FORWARD") {
      motor.forward(1.0);  // Move forward at full speed
      last_move = 1;
    } else if (cmd == "BACKWARD") {
      motor.backward(1.0);  // Move backward at full speed
      last_move = 2;
    } else if (cmd == "LEFT") {
      motor.turnLeft(0.5);  // Turn left
      last_move = 3;
    } else if (cmd == "RIGHT") {
      motor.turnRight(0.5);  // Turn right
      last_move = 4;
    } else {
      motor.setSpeed(0, 0);  // Stop
      last_move = 0;
    }
}

// Request information handling
void requestInfo(String cmd) {
  distance = sr04.Distance();  // Get current distance
  
  noInterrupts();
  long currentBCount = B_wheel_pulse_count;  // Get current B wheel encoder count
  long currentACount = A_wheel_pulse_count;  // Get current A wheel encoder count
  interrupts();

  display.clear();
  display.drawString(0, 0, "Distance: " + String(distance) + "cm");
  display.drawString(0, 15, "A Pulse: " + String(currentACount));
  display.drawString(0, 30, "B Pulse: " + String(currentBCount));
  display.drawString(0, 45, cmd);
  display.display();
  if (cmd == "REQUEST_INFO") {
    delay(1000);  // Delay 1 second
    String frame = createDataFrame(distance, currentACount, currentBCount);
    Serial1.println(frame);  // Send data via serial port
  } 
}

// Command parsing
volatile State parsedState = START_PAGE; // Default state

CommandInfo parseCommand(String fullCommand) {
    // Split received string, assuming fullCommand is comma-separated
    int firstCommaIndex = fullCommand.indexOf(',');
    int secondCommaIndex = fullCommand.indexOf(',', firstCommaIndex + 1);
    int thirdCommaIndex = fullCommand.indexOf(',', secondCommaIndex + 1);
    int fourthCommaIndex = fullCommand.indexOf(',', thirdCommaIndex + 1);

    String carIdStr = fullCommand.substring(0, firstCommaIndex);
    String modeStr = fullCommand.substring(firstCommaIndex + 1, secondCommaIndex);
    String startStr = fullCommand.substring(secondCommaIndex + 1, thirdCommaIndex);
    String stateStr = fullCommand.substring(thirdCommaIndex + 1, fourthCommaIndex);
    String commandStr = fullCommand.substring(fourthCommaIndex + 1);

    // Convert strings to appropriate data types
    int carId = carIdStr.toInt();
    int mode = modeStr.toInt();
    int start = startStr.toInt();

    if (stateStr == "START") parsedState = START_PAGE;
    else if (stateStr == "CONFIG") parsedState = REMOTE_CONTROL;
    else if (stateStr == "REMOTE") parsedState = REMOTE_CONTROL;
    else if (stateStr == "REQUEST") parsedState = INFO_COLLECTION;
    else if (stateStr == "DISTANCE") parsedState = DISTANCE_PID;

    return {carId, mode, start, parsedState, commandStr};
}

void setup() {
  motor.computePID();  // Initial PID computation
  motor.setSpeed(0, 0); // Initial speed set to 0
  Serial1.begin(BAUD_RATE, SERIAL_8N1, 40, 1);
  while (!Serial1) {
    ; // Wait for serial connection
  }
  pinMode(BENCB, INPUT_PULLUP); // Set encoder pins as pull-up input
  pinMode(BENCA, INPUT_PULLUP);
  pinMode(AENCB, INPUT_PULLUP);
  pinMode(AENCA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, CHANGE); // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, CHANGE);

  display.init();  // Initialize display
  display.flipScreenVertically();  // Flip display orientation
  display.clear();  // Clear screen
  display.setFont(ArialMT_Plain_10);  // Set font
  display.setTextAlignment(TEXT_ALIGN_LEFT);  // Set text alignment

  lastTime = millis();  // Record current time
}

volatile bool pidActive = false; // PID control activation flag

struct PulseRecord {
    long A_wheel_delta;
    long B_wheel_delta;
};
const int MAX_RECORDS = 10000;  // Assume we can store up to 10000 records
PulseRecord pulseRecords[MAX_RECORDS];
int recordIndex = 0;

long last_A_wheel_pulse_count = 0;
long last_B_wheel_pulse_count = 0;

void loop() {
  display.clear();
  String command = Serial1.readStringUntil('\n');  // Read a line of command
  CommandInfo cmdInfo = parseCommand(command);  // Parse command
  currentState = cmdInfo.state;
  display.drawString(0, 45, command);
  long currentA = A_wheel_pulse_count;
  long currentB = B_wheel_pulse_count;
  
  switch (currentState) {
    case START_PAGE:
      motor.setSpeed(0, 0); // Stop motors
      pidActive = false;
      display.drawString(0, 0, "Start Page");
      display.drawString(0, 15, "Car ID: "+ String(CarID));
      display.drawString(0, 30, "Mode: "+ String(cmdInfo.mode) + "Start: "+ String(cmdInfo.start));
      break;
    case REMOTE_CONTROL:
      if(CarID == cmdInfo.carID){
        display.drawString(0, 0, "Remote Control Mode");
        if((last_move == 1 && cmdInfo.command == "FORWARD") || (last_move == 2 && cmdInfo.command == "BACKWARD")){
          if (abs(currentA - last_A_wheel_pulse_count) >= 1000 || abs(currentB - last_B_wheel_pulse_count) >= 1000) {
          // Calculate pulse difference from last record
          long deltaA = currentA - last_A_wheel_pulse_count;
          long deltaB = currentB - last_B_wheel_pulse_count;

          // Store pulse difference
          if (recordIndex < MAX_RECORDS) {  // Ensure not to exceed array boundary
              pulseRecords[recordIndex].A_wheel_delta = deltaA;
              pulseRecords[recordIndex].B_wheel_delta = deltaB;
              recordIndex++;  // Update record index
          }

          // Update last pulse counts
          last_A_wheel_pulse_count = currentA;
          last_B_wheel_pulse_count = currentB;
          }
        }
        else if((last_move == 1 && cmdInfo.command == "LEFT") || (last_move == 2 && cmdInfo.command == "RIGHT") || (last_move == 3 && cmdInfo.command == "RIGHT") || (last_move == 4 && cmdInfo.command == "LEFT")){
          long deltaA = currentA - last_A_wheel_pulse_count;
          long deltaB = currentB - last_B_wheel_pulse_count;
          // Store pulse difference
          if (recordIndex < MAX_RECORDS) {  // Ensure not to exceed array boundary
              pulseRecords[recordIndex].A_wheel_delta = deltaA;
              pulseRecords[recordIndex].B_wheel_delta = deltaB;
              recordIndex++;  // Update record index
          }
        }
        else if((last_move == 3 && cmdInfo.command == "LEFT") || (last_move == 4 && cmdInfo.command == "RIGHT")){
          if (abs(currentA - last_A_wheel_pulse_count) >= 1000 || abs(currentB - last_B_wheel_pulse_count) >= 1000) {
          // Calculate pulse difference from last record
          long deltaA = currentA - last_A_wheel_pulse_count;
          long deltaB = currentB - last_B_wheel_pulse_count;

          // Store pulse difference
          if (recordIndex < MAX_RECORDS) {  // Ensure not to exceed array boundary
              pulseRecords[recordIndex].A_wheel_delta = deltaA;
              pulseRecords[recordIndex].B_wheel_delta = deltaB;
              recordIndex++;  // Update record index
          }

          // Update last pulse counts
          last_A_wheel_pulse_count = currentA;
          last_B_wheel_pulse_count = currentB;
          }
        }
        handleCommand(cmdInfo.command);
      }
      break;

    case INFO_COLLECTION:
      if(CarID == cmdInfo.carID){
        display.drawString(0, 0, "Information Collection Mode");
        display.drawString(0, 15, "Wheel and Distance Data");
        requestInfo(cmdInfo.command);
      }
      break;
    case DISTANCE_PID:
      display.drawString(0, 0, "Distance PID Control Mode");
      String set_command = cmdInfo.command;
      if (set_command.startsWith("SET ")) {
          setPoint = set_command.substring(4).toDouble();
      }
      
      if (set_command == "Start" && CarID == cmdInfo.carID) {
          pidActive = true;
          display.drawString(0, 30, "PID Started");
          display.display();
      }
      display.drawString(0, 15, "Distance "+ String(setPoint) + "cm");

      if (pidActive) {
          computePID();
      }
    break;
  }
  display.display();
}
