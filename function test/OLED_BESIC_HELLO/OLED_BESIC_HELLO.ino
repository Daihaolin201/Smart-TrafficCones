#include <Wire.h>               // Include Wire library for I2C communication
#include "SSD1306Wire.h"        // Include the library for the SSD1306 OLED display driver

// Initialize the OLED display, using a common I2C OLED display as an example
// The address is usually 0x3C or 0x3D, and SDA and SCL pins depend on your Arduino model
SSD1306Wire display(0x3c, 21, 47);

void setup() {
  // Initialize the serial port for debugging
  Serial.begin(115200);
  
  // Initialize the display
  display.init();
  
  // Clear the screen
  display.clear();
  
  // Set the font, using a built-in font here
  display.setFont(ArialMT_Plain_10);
  
  // Set text alignment, here set to left align
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  // Display "Hello World" on the screen
  display.drawString(0, 0, "Hello world");
  
  // Display the contents of the buffer on the screen
  display.display();
}

void loop() {
  // No actions needed in the loop for this example
}
