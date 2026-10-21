#include "stepper_motor_class.h"

StepperMotor::StepperMotor(int enPin, int stepPin, int dirPin, int diagPin, int motorAddr, Stream &serialPort)
    : enPin(enPin), 
      stepPin(stepPin), 
      dirPin(dirPin), 
      diagPin(diagPin), 
      motorAddr(motorAddr),
      driver(&serialPort, R_SENSE, motorAddr),
      stepper(AccelStepper::DRIVER, stepPin, dirPin)
{
}

void StepperMotor::run()
{
  stepper.run();
}

bool StepperMotor::testConnection()
{
  return driver.test_connection() == 0 ? true : false;
}

void StepperMotor::move(int steps)
{
  stepper.move((long)(steps * STEPS_PER_REV));
}

uint16_t StepperMotor::getCurrentPosition()
{
  return driver.MSCNT();
}

void StepperMotor::setup()
{
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);     // active low -- start disabled
  pinMode(diagPin, INPUT);

  driver.begin();
  driver.toff(4);                 // must be nonzero or the driver stays off
  driver.blank_time(24);
  driver.rms_current(RUN_CURRENT);
  driver.microsteps(MICROSTEPS);
  driver.intpol(true);            // interpolate to 256 internally
  driver.en_spreadCycle(false);   // false = StealthChop (quiet)
  driver.pwm_autoscale(true);     // required for StealthChop
  driver.TCOOLTHRS(0xFFFFF);      // enable StallGuard across the speed range
  driver.SGTHRS(50);              // stall threshold -- needs tuning

  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1500);

  digitalWrite(enPin, LOW);      // energize
}
