# SoC Estimation

The Arduino C program runs on the ESP32 board to estimate the SoC of a 24V battery. It also sends all the related data via MQTT, by which a remote laptop/desktop can read this data and store it in a CSV file using a python script running on the system.


# Software Requirements

 - Arduino IDE with ESP32 board installed.
 - Python 3.8+ installed on the remote system.

## C Libraries in Arduino IDE

 - BasicLinearAlgebra.h
 - Wire.h
 - WiFi.h
 - WebServer.h
 - Adafruit_ADS1X15.h
 - Preferences.h
 - PubSubClient.h
 - ArduinoJson.h

## Python Libraries

- JSON
 - Paho MQTT
 - Pandas

