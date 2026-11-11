#include <Arduino.h>
#include "stepper_motor_class.h"
#include "arm.h"

bool done = false; 
Arm arm;
String inputBuffer = "";   // global, next to Arm arm;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);          // TMC2209 UART (stepper motor drivers serial communication)

  Serial.println("Beginning arm test");
 
  if (!arm.armSetup())
  {
    Serial.println("Stepper driver commds FAILED");
    return;
  }
}

void loop() {
  arm.runWrist();                          // step motors every pass, as fast as possible

  while (Serial.available()) {             // read whatever bytes have arrived
    char c = Serial.read();
    if (c == '\n') {                       // full line received
      if (arm.parseSerial(inputBuffer)) {  // sets targets + calls updateMotors()
        done = false;                      // new move -> report again when finished
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }

  if (!done && arm.wristDone()) {          // report once when the move finishes
    WristPosition pos = arm.getPosition();
    Serial.print("Move complete: ");
    Serial.print(pos.stepper1);
    Serial.print(", ");
    Serial.println(pos.stepper2);
    done = true;
  }
}