//https://store.techmaze.ae/product/99187748
//need to use arduino mega doesnt work on due
#include "SoftwareSerial.h"

// Define the software serial object
SoftwareSerial gpsSerial(10, 11); // RX, TX (Connect GT-U7 RX to Arduino TX and GT-U7 TX to Arduino RX)

void setup() {
  // Start the software serial communication
  gpsSerial.begin(9600);
  // Start the serial communication with the computer
  Serial.begin(9600);
}

void loop() {
  if (gpsSerial.available()) {
    
    char c = gpsSerial.read();
    
    // Check if the character received is the start of a NMEA sentence
    if (c == '$') {
      String sentence = gpsSerial.readStringUntil('\n');
//      Serial.println(sentence);
      // Check if the sentence starts with "GPGGA" (GPS fix data)
      if (sentence.startsWith("GPGGA")) {
        // Print the GPS data to the serial monitor
        //prints time, latitude, N or S, Longitude, E or W, GPS quality indicator, Number of satellites, Antenna altitude,  meters
        Serial.println(sentence);
      }
    }
  }
}
