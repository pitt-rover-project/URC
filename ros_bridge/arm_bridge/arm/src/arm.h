#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include "stepper_motor_class.h"

const float deadband = 1.0;

struct WristPosition {
  float stepper1;
  float stepper2;
};

class Arm {
  public:
    Arm();
    bool armSetup();
    void runWrist();
    WristPosition getPosition();
    bool parseSerial(String message);
    bool writeSerial(String message);
    bool wristDone();

  private:
    float pitchTarget;
    float rollTarget;
    float gripperTarget;
    void updateMotors();
    StepperMotor stepper1;
    StepperMotor stepper2;
};

#endif