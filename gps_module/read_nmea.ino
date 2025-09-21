#include <SoftwareSerial.h>
#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;

// SoftwareSerial for GPS module
SoftwareSerial gpsSerial(10, 11); // RX, TX

void setup()
{
  Serial.begin(115200);          // Debug to PC
  while (!Serial);
  Serial.println("u-blox NMEA Data Reader");

  gpsSerial.begin(38400);       // UART1 connected to ZED-F9P

  // Connect to the u-blox module using UART1
  if (myGPS.begin(gpsSerial) == false)
  {
    Serial.println(F("u-blox GPS not detected on UART1. Please check wiring."));
    while (1);
  }
   
  // Set UART1 to output only NMEA messages
  myGPS.setUART1Output(COM_TYPE_NMEA);
  
  //Save configuration to module's flash memory
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  
}

void loop()
{
  // Forward all NMEA data from GPS to Serial Monitor
  while (gpsSerial.available())
  {
    Serial.write(gpsSerial.read());
  }
}