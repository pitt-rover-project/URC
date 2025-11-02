#include <stdlib.h>
#define PWM1Front 3 //all 1's are on right side of robot
#define PWM1Middle 5
#define PWM1Back 7
#define DIR1Front 52
#define DIR1Middle 50
#define DIR1Back 48
#define PWM2Front 2 //all 2's are on the left side or robot
#define PWM2Middle 4
#define PWM2Back 6
#define DIR2Front 53
#define DIR2Middle 51
#define DIR2Back 49

// REPLACE
#define WHEEL_BASE 0.7384    // distance between the left and right wheels in meters
#define MAX_LINEAR_SPEED 1.0    // maximum linear speed in m/s
#define MAX_PWM 255

void setup()
{
  Serial.begin(115200);
  Serial.println("in setup");

  pinMode(DIR1Front, OUTPUT);
  pinMode(DIR1Middle, OUTPUT);
  pinMode(DIR1Back, OUTPUT);
  pinMode(DIR2Front, OUTPUT);
  pinMode(DIR2Middle, OUTPUT);
  pinMode(DIR2Back, OUTPUT);
  pinMode(PWM1Front, OUTPUT);
  pinMode(PWM1Middle, OUTPUT);
  pinMode(PWM1Back, OUTPUT);
  pinMode(PWM2Front,   OUTPUT);
  pinMode(PWM2Middle,   OUTPUT);
  pinMode(PWM2Back,   OUTPUT);
}




void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    float linear_x, linear_y, linear_z;
    float angular_x, angular_y, angular_z;

    int parsed = sscanf(data.c_str(), "%f,%f,%f,%f,%f,%f",
                        &linear_x, &linear_y, &linear_z,
                        &angular_x, &angular_y, &angular_z);



    if (parsed == 6) {
      Serial.print("Received: ");
      Serial.print("Linear (");
      Serial.print(linear_x); Serial.print(", ");
      Serial.print(linear_y); Serial.print(", ");
      Serial.print(linear_z); Serial.print(") | ");
      Serial.print("Angular (");
      Serial.print(angular_x); Serial.print(", ");
      Serial.print(angular_y); Serial.print(", ");
      Serial.print(angular_z); Serial.println(")");

      float left_speed = linear_x - (angular_z * (WHEEL_BASE));
      float right_speed = linear_x + (angular_z * (WHEEL_BASE));

      int pwm_left  = (int)(min(abs(left_speed) / MAX_LINEAR_SPEED, 1.0) * MAX_PWM);
      int pwm_right = (int)(min(abs(right_speed) / MAX_LINEAR_SPEED, 1.0) * MAX_PWM);

      if (left_speed >= 0) {
        digitalWrite(DIR2Front, HIGH);
        digitalWrite(DIR2Middle, HIGH);
        digitalWrite(DIR2Back, HIGH);
      } else {
        digitalWrite(DIR2Front, LOW);
        digitalWrite(DIR2Middle, LOW);
        digitalWrite(DIR2Back, LOW);
      }

      // Set directions based on the sign of the speed for the right side motors
      if (right_speed >= 0) {
        digitalWrite(DIR1Front, HIGH);
        digitalWrite(DIR1Middle, HIGH);
        digitalWrite(DIR1Back, HIGH);
      } else {
        digitalWrite(DIR1Front, LOW);
        digitalWrite(DIR1Middle, LOW);
        digitalWrite(DIR1Back, LOW);
      }

      // Apply the PWM values to the motors
      analogWrite(PWM2Front, pwm_left);
      analogWrite(PWM2Middle, pwm_left);
      analogWrite(PWM2Back, pwm_left);
      analogWrite(PWM1Front, pwm_right);
      analogWrite(PWM1Middle, pwm_right);
      analogWrite(PWM1Back, pwm_right);
    }
  }


  Serial.flush();
}
