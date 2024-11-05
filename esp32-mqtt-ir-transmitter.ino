#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <IRremote.h>  // Include the IRremote library for IR communication

#define IR_LED_PIN 5  // Pin connected to IR transmitter LED (GPIO 5)
#define DISTANCE_THRESHOLD 50  // Threshold in mm for detecting presence of an object

Adafruit_VL53L0X lox;   // Create an instance of the Adafruit_VL53L0X sensor
IRsend irsend(IR_LED_PIN);  // Create an instance of the IR sender, connected to GPIO 5

bool itemDetected = false;   // Flag to track if an item was detected in front of the sensor

// IR Command to send when the item is detected or removed
unsigned long pwrCommand = 0x2FD48B7;

void setup() {
  // Start serial communication at 115200 baud rate
  Serial.begin(115200);

  // Wait until Serial Monitor is connected (useful for ESP32)
  while (!Serial) {
    // Wait for the serial port to be ready
  }

  // Initialize the VL53L0X sensor
  if (!lox.begin()) {
    Serial.println("Failed to initialize VL53L0X sensor!");
    while (1);  // Halt if the sensor fails to initialize
  } else {
    // Success message if the sensor is initialized correctly
    Serial.println("VL53L0X sensor initialized successfully!");
  }

  // Initialize the IR sender
  irsend.enableIROut(38);  // Set frequency to 38kHz (standard for many IR devices)

  // Provide initial message
  Serial.println("IR Command on Distance Change Sketch Started");
}

void loop() {
  // Get the current distance from the VL53L0X sensor
  uint16_t currentDistance = lox.readRange();

  if (lox.timeoutOccurred()) {
    Serial.println("Sensor timeout!");
    return;
  }

  // Debugging: Print the distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(currentDistance);
  Serial.println(" mm");

  // If the distance changes significantly and an object is detected or removed
  if (currentDistance < DISTANCE_THRESHOLD && !itemDetected) {
    // Object detected in front of the sensor
    Serial.println("Object detected!");
    irsend.sendNEC(pwrCommand, 32);  // Send IR command (NEC format)
    itemDetected = true;
  }
  else if (currentDistance >= DISTANCE_THRESHOLD && itemDetected) {
    // Object removed from in front of the sensor
    Serial.println("Object removed!");
    irsend.sendNEC(pwrCommand, 32);  // Send IR command (NEC format)
    itemDetected = false;
  }

  // Wait for 500 milliseconds before the next measurement
  delay(500);  // Delay 500 ms before taking the next measurement
}
