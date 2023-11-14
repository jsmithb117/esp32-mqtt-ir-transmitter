# esp32-mqtt-ir-transmitter
Control your infrared-controlled devices with WiFi using an esp32 and IR transmitter module.

### Time Of Flight sensor
  Detects whether XBox One controller is in front of sensor.
  Sends IR PWR command on state change.
  
### IR Receiver
  Relays all IR commands to MQTT.
  Disabled during IR Transmit events and TOF sensor

### IR Transmitter
  When this device receives an MQTT subscription message, IR Transmitter transmits the message
  Also sends the PWR command when TOF sensor state changes.

### Temp/Humidity/Barometric pressure sensors
  Expands monitoring of the household.
  Sends raw data to MQTT server.
