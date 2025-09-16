/*
 * RC Car - Gamepad Control - TRANSMITTER
 * This sketch receives formatted data strings from a Python script via USB Serial,
 * parses the data, and transmits it to the RC car using an NRF24L01 module.
 * It also sends a success/fail status back to the Python script.
 *
 * Expected Serial Input Format: "X:value,Y:value\n" (e.g., "X:512,Y:-250\n")
 *
 * Wiring (ESP32 -> NRF24L01):
 * VCC  -> 3V3
 * GND  -> GND
 * CSN  -> D5
 * CE   -> D4
 * MOSI -> D23
 * MISO -> D19
 * SCK  -> D18
 */

#include <SPI.h>
#include <RF24.h>

// A structure to hold the control data
struct ControlData {
  int steer; // Value for steering (-512 to 512)
  int throttle; // Value for throttle (-512 to 512)
};

ControlData controlData;

RF24 radio(4, 5); // CE, CSN
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  Serial.println("Gamepad Transmitter Initialized");

  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1) {} // Freeze
  }
  
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH); // Must match receiver: HIGH power for better range
  radio.setDataRate(RF24_250KBPS); // Must match receiver: Lower data rate for better reliability
  radio.setChannel(76);
  radio.setAutoAck(true); // Enable auto acknowledgments for reliability
  radio.enableAckPayload(); // Enable acknowledgment payloads
  radio.setRetries(5, 15); // Must match receiver: More retries with longer delay
  radio.stopListening();
}

void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    
    // Parse the incoming string: "X:value,Y:value"
    int xPos = inputString.indexOf("X:");
    int yPos = inputString.indexOf("Y:");
    int commaPos = inputString.indexOf(',');

    if (xPos != -1 && yPos != -1 && commaPos != -1) {
      String xValueStr = inputString.substring(xPos + 2, commaPos);
      String yValueStr = inputString.substring(yPos + 2);
      
      controlData.steer = xValueStr.toInt();
      controlData.throttle = yValueStr.toInt();
      
      // Send the data structure via radio
      if (radio.write(&controlData, sizeof(controlData))) {
        Serial.println("TX SUCCESS"); // Send feedback to Python
      } else {
        Serial.println("TX FAILED"); // Send feedback to Python
      }
    }
  }
}
