int channel_PWM = 0;  // Define the PWM channel number
int freq_PWM = 2000;  // Define the frequency of the PWM signal (in Hz)
int resolution_PWM = 8;  // Define the resolution of the PWM signal (bit depth)
const int PWM_Pin = 4;  // Define the pin number used for PWM output

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  ledcSetup(channel_PWM, freq_PWM, resolution_PWM);  // Configure the frequency and resolution of the PWM channel
  ledcAttachPin(PWM_Pin, channel_PWM);  // Attach the PWM channel to the specified pin
  pinMode(15, OUTPUT);  // Set pin 15 as an output mode
  pinMode(18, OUTPUT);  // Set pin 18 as an output mode
  digitalWrite(15, HIGH);  // Set pin 15 to high
  digitalWrite(18, LOW);   // Set pin 18 to low
}

void get_pwm_info() {
  Serial.println("*******************************************************************");  // Print a separator line to separate different information
  Serial.print("Duty cycle of specified channel: ");  // Print the duty cycle of the specified channel
  Serial.println(ledcRead(channel_PWM));  // Print the duty cycle value of the specified channel
  Serial.print("Frequency of specified channel: ");  // Print the frequency of the specified channel
  Serial.println(ledcReadFreq(channel_PWM));  // Print the frequency value of the specified channel
}

void loop() {
  ledcWrite(channel_PWM, 128);  // Set the duty cycle of the specified PWM channel to 128, half of full scale
  get_pwm_info();  // Call the function to get PWM information, print the current PWM settings
  delay(5000);  // Delay for 5 seconds
  //ledcDetachPin(PWM_Pin);  // Detach the PWM channel from the pin, free the pin for other uses
}
