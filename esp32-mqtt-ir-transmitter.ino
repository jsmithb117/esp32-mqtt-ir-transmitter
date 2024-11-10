#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

// MUST USE Arduino V1.06 for IRremoteESP8266 library to work correctly.  This precludes newer Boards (S3, C6...)
const int irPin = 5; // GPIO pin connected to the DAT pin of the IR transmitter
const int targetDistance = 50; // Target distance in mm for detection
const uint32_t pwrCommand = 0x2FD48B7;
static bool objectInRange; // Tracks if an object is currently within range
const int debounceDelayMS = 1000;
const int loopDelay = 500;
const int successStatus = 4;
const bool debugStatus = false;


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
IRsend irsend(irPin);
VL53L0X_RangingMeasurementData_t measure;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize the VL53L0X
  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X! Check wiring."));
    while (1);
  }
  Serial.println(F("VL53L0X initialized."));

  // Initialize the IR transmitter
  irsend.begin();
}

void loop() {
  // Take a measurement
  lox.rangingTest(&measure, debugStatus);

  if (measure.RangeStatus != successStatus) {
    int currentDistance = measure.RangeMilliMeter;
    Serial.print(F("Distance: "));
    Serial.print(currentDistance);
    Serial.println(F(" mm"));

    // Check if an object has entered within target distance
    if (currentDistance <= targetDistance && !objectInRange) {
      Serial.println(F("Object detected. Sending IR command."));
      irsend.sendNEC(pwrCommand, 32); // Send the IR command
      objectInRange = true; // Mark object as in range
      delay(debounceDelayMS); // Delay to avoid immediate re-trigger
    }
    // Check if an object has been removed from the target range
    else if (currentDistance > targetDistance && objectInRange) {
      Serial.println(F("Object removed. Sending IR command."));
      irsend.sendNEC(pwrCommand, 32); // Send the IR command
      objectInRange = false; // Mark object as out of range
      delay(debounceDelayMS); // Delay to avoid immediate re-trigger
    }
  } else {
    Serial.println(F("Out of range"));
  }

  delay(loopDelay); // Small delay between measurements
}
