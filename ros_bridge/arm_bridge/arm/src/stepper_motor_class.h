#ifndef STEPPER_MOTOR_CLASS_H
#define STEPPER_MOTOR_CLASS_H

#include <TMCStepper.h>
#include <AccelStepper.h>

#define R_SENSE   0.11f   // BTT TMC2209 sense resistors
#define RUN_CURRENT 400              // mA RMS
#define MICROSTEPS 8
#define STEPS_PER_REV (200L * MICROSTEPS)

class StepperMotor {
  public:
    StepperMotor(int enPin, int stepPin, int dirPin, int diagPin, int motorAddr, Stream &serialPort);
    void run();
    void setup();
    bool testConnection();
    void move(int steps);

  private:
    int enPin;
    int stepPin;
    int dirPin;
    int diagPin;
    int motorAddr;
    TMC2209Stepper driver;
    AccelStepper stepper;
};

#endif