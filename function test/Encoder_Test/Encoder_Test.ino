#include <Arduino.h>

// === MOTOR PIN DEFINITIONS ===

// Define the pins for A/B encoders. Each encoder uses two signal lines connected to corresponding Hall elements.
// For a single encoder: The motor speed is calculated by detecting the frequency of voltage level changes of one Hall element,
// and the rotation direction is determined by the voltage level state of the other Hall sensor.

// Pin definitions for encoder A
const uint16_t AENCA = 36;        // Encoder A input A_C2(B)
const uint16_t AENCB = 3;         // Encoder A input A_C1(A)

// Pin definitions for encoder B
const uint16_t BENCB = 35;        // Encoder B input B_C2(B)
const uint16_t BENCA = 39;        // Encoder B input B_C1(A)

// Used to calculate the number of high-to-low voltage transitions of a Hall sensor of the encoder within an "interval" time in milliseconds.
// Since the interrupt initialization later uses RISING, specifically, it counts the number of transitions from low to high.
volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count = 0;

// Period time for calculating speed, speed is calculated every this many milliseconds.
int interval = 100;

// Current time
long currentMillis = 0;

// To store the speeds for both sides
double rpm_B = 0;
double rpm_A = 0;

// Gear reduction ratio of the motor, the motor speed is different from the output shaft speed.
// For example, with a DCGM3865 motor, the reduction ratio is 1:42, meaning the motor spins 42 revolutions for one revolution of the output shaft.
// The higher the number of motor revolutions corresponding to one revolution of the output shaft, the higher the reduction ratio and usually the greater the torque.
double reduction_ratio = 42;

// Number of pulses per revolution of the encoder, number of high and low voltage changes of a Hall sensor when the motor spins one revolution.
int ppr_num = 11;

// Number of pulses per revolution of the output shaft, derived by multiplying the reduction ratio by the encoder's pulses per revolution.
double shaft_ppr = reduction_ratio * ppr_num;

// IRAM_ATTR is a macro used to decorate functions and variables.
// It indicates that the function or variable is placed in IRAM (Instruction RAM), which is the instruction memory of the ESP32 chip.
// Functions and variables decorated with IRAM_ATTR are placed in IRAM by the compiler,
// which can increase their execution speed, especially for frequently executed sections of code.
// This is very useful in applications requiring high real-time performance.

// Interrupt callback function, see the attachInterrupt() function below. When the voltage level of a Hall encoder changes from low to high,
// this function is called. In this function, the voltage level of the other Hall element is checked to determine the direction of rotation.
void IRAM_ATTR B_wheel_pulse() {
  if(digitalRead(BENCA)){
    B_wheel_pulse_count++;
  }
  else{
    B_wheel_pulse_count--;
  }
}

void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){
    A_wheel_pulse_count++;
  }
  else{
    A_wheel_pulse_count--;
  }
}

void setup(){
  // Set the working mode of the encoder-related pins
  // When using encoders, it's often necessary to enable pull-up resistors on their pins because encoders typically use open-drain or passive (open collector) outputs.

  // When the encoder pins are configured as inputs, without an external pull-up resistor or an internal pull-up resistor (using the INPUT_PULLUP option) connected to the pins,
  // the pin level might drift or be in an uncertain state. This could lead to erroneous or unstable readings of the encoder signals.

  // By using the INPUT_PULLUP option, you can enable an internal pull-up resistor on the pins, setting the default level of the pins to high (logic 1).
  // This effectively prevents pin level drift and ensures that the pins remain in a defined state when there are no external signals. When the encoder generates a signal,
  // the pin level will change, and this change can be detected by interrupts or polling.

  // Therefore, using the INPUT_PULLUP option when configuring encoder pins is a common practice to enhance the reliability and stability of the encoder signals.
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);
  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);

  // Set up interrupts and corresponding callback functions, when BENCB changes from low to high (RISING), call B_wheel_pulse function.
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  // Set up interrupts and corresponding callback functions, when AENCB changes from low to high (RISING), call A_wheel_pulse function.
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  // Initialize the serial port with a baud rate of 115200
  Serial.begin(115200);
  // Wait for serial port to initialize
  while(!Serial){}
}

void loop(){
  // Calculate the output shaft RPM for channel B, in revolutions per minute
  rpm_B = (float)((B_wheel_pulse_count / shaft_ppr) * 60 * (1000 / interval));
  B_wheel_pulse_count = 0;

  // Calculate the output shaft RPM for channel A, in revolutions per minute
  rpm_A = (float)((A_wheel_pulse_count / shaft_ppr) * 60 * (1000 / interval));
  A_wheel_pulse_count = 0;

  Serial.print("RPM_A: "); Serial.print(rpm_A); Serial.print("   RPM_B: "); Serial.println(rpm_B);
  Serial.println("--- --- ---");

  delay(interval);
}
