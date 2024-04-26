const uint16_t AENCA = 36;  // Encoder A input A_C2(B)
const uint16_t AENCB = 3;   // Encoder A input A_C1(A)
const uint16_t BENCB = 35;  // Encoder B input B_C2(B)
const uint16_t BENCA = 39;  // Encoder B input B_C1(A)

volatile long B_wheel_pulse_count = 0;
volatile long A_wheel_pulse_count = 0;

void IRAM_ATTR B_wheel_pulse() {
  // Check the second Hall sensor signal of channel B to determine the direction of rotation
  if (digitalRead(BENCA) == HIGH) {
    B_wheel_pulse_count++;
  } else {
    B_wheel_pulse_count--;
  }
}

void IRAM_ATTR A_wheel_pulse() {
  // Check the second Hall sensor signal of channel A to determine the direction of rotation
  if (digitalRead(AENCA) == HIGH) {
    A_wheel_pulse_count++;
  } else {
    A_wheel_pulse_count--;
  }
}

void setup() {
  pinMode(BENCB, INPUT_PULLUP);
  pinMode(BENCA, INPUT_PULLUP);
  pinMode(AENCB, INPUT_PULLUP);
  pinMode(AENCA, INPUT_PULLUP);

  // Attach an interrupt to the main signal line to detect changes
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);

  Serial.begin(115200);
  while (!Serial) {}  // Wait for the serial port to connect
}

void loop() {
  // Safely read pulse count values
  noInterrupts();
  long currentBCount = B_wheel_pulse_count;
  long currentACount = A_wheel_pulse_count;
  interrupts();

  // Print the current cumulative pulse counts
  Serial.print("Current A wheel pulse count: ");
  Serial.println(currentACount);
  Serial.print("Current B wheel pulse count: ");
  Serial.println(currentBCount);

  delay(1000); // Update once per second
}
