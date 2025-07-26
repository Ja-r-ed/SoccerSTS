#include <IRremote.hpp>
#include <Arduino.h>
#define DECODE_NEC
#include "PinDefinitionsAndMore.h"

int M1a = 7;
int M1b = 6;  
int M2a = 4;
int M2b = 5;  
int M3a = 10;
int M3b = 11;  
int M4a = 2;
int M4b = 3;  

int Da = 11;
int Db = 12;
int DPWM = 10;  

int GlobalPWM = 40;
//   Front Left  (M1)
//   Front Right (M2)
//   Back Left   (M3)
//   Back Right  (M4)

//
void setup() {
  IrReceiver.begin(A0, ENABLE_LED_FEEDBACK);
  pinMode(M1a, OUTPUT);
  pinMode(M1b, OUTPUT);
  pinMode(M2a, OUTPUT);
  pinMode(M2b, OUTPUT);
  pinMode(M3a, OUTPUT);
  pinMode(M3b, OUTPUT);
  pinMode(M4a, OUTPUT);
  pinMode(M4b, OUTPUT);
}

void loop() {

  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      IrReceiver.printIRResultRawFormatted(&Serial, true);
      IrReceiver.resume();  // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
    } else {
      IrReceiver.resume();  // Early enable receiving of the next IR frame
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
    }
    Serial.println();
    if (IrReceiver.decodedIRData.command == 0x18) {  // forwards
      drive(0, 0.2, 0);
    } else if (IrReceiver.decodedIRData.command == 0x52) {  // backwards
      drive(180, 0.2, 0);
    } else if (IrReceiver.decodedIRData.command == 0x5A) {  // right
      drive(90, 0.2, 0);
    } else if (IrReceiver.decodedIRData.command == 0x8) {  // left
      drive(270, 0.2, 0);
    } else if (IrReceiver.decodedIRData.command == 0x45) {  // turn left
      digitalWrite(M1a, LOW);
      analogWrite(M1b, GlobalPWM);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, GlobalPWM);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, GlobalPWM);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, GlobalPWM);
    } else if (IrReceiver.decodedIRData.command == 0x46) {  // dribble
      digitalWrite(Da, HIGH);
      digitalWrite(Db, LOW);
      analogWrite(DPWM, 40);
    } else if (IrReceiver.decodedIRData.command == 0x47) {  // turn right
      digitalWrite(M1b, LOW);
      analogWrite(M1a, GlobalPWM);

      digitalWrite(M2b, LOW);
      analogWrite(M2a, GlobalPWM);

      digitalWrite(M3b, LOW);
      analogWrite(M3a, GlobalPWM);

      digitalWrite(M4b, LOW);
      analogWrite(M4a, GlobalPWM);
    }
    delay(100);
    digitalWrite(M1a, LOW);
    digitalWrite(M1b, LOW);
    digitalWrite(M2a, LOW);
    digitalWrite(M2b, LOW);
    digitalWrite(M3a, LOW);
    digitalWrite(M3b, LOW);
    digitalWrite(M4a, LOW);
    digitalWrite(M4b, LOW);
  }
}

void MotorTest() {
  analogWrite(M1a, 120);
  digitalWrite(M1b, LOW);

  analogWrite(M2a, 120);
  digitalWrite(M2b, LOW);

  analogWrite(M3a, 120);
  digitalWrite(M3b, LOW);

  analogWrite(M4a, 120);
  digitalWrite(M4b, LOW);
}

void dribble() {
  digitalWrite(Da, HIGH);
  digitalWrite(Db, LOW);
  analogWrite(DPWM, 20);
}

void SetSpeedPID(int Motor, int PWM) {
  static int PWM1Value = 0;
  static int PWM2Value = 0;
  static int PWM3Value = 0;
  static int PWM4Value = 0;

  if (Motor == 1) {
    if (PWM > 0) {
      analogWrite(M1a, pidController(PWM - PWM1Value));
      digitalWrite(M1b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M1a, LOW);
      analogWrite(M1b, pidController(PWM - PWM1Value));
    }
    if (PWM = 0) {
      digitalWrite(M1a, LOW);
      digitalWrite(M1b, LOW);
    }
    int PWM1Value = pidController(PWM - PWM1Value);
  }

  if (Motor == 2) {
    if (PWM > 0) {
      analogWrite(M2a, pidController(PWM - PWM2Value));
      digitalWrite(M2b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M2a, LOW);
      analogWrite(M2b, pidController(PWM - PWM2Value));
    }
    if (PWM = 0) {
      digitalWrite(M2a, LOW);
      digitalWrite(M2b, LOW);
    }
    int PWM2Value = pidController(PWM - PWM2Value);
  }

  if (Motor == 3) {
    if (PWM > 0) {
      analogWrite(M3a, pidController(PWM - PWM3Value));
      digitalWrite(M3b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M3a, LOW);
      analogWrite(M3b, pidController(PWM - PWM3Value));
    }
    if (PWM = 0) {
      digitalWrite(M3a, LOW);
      digitalWrite(M3b, LOW);
    }
    int PWM3Value = pidController(PWM - PWM3Value);
  }

  if (Motor == 4) {
    if (PWM > 0) {
      analogWrite(M4a, pidController(PWM - PWM4Value));
      digitalWrite(M4b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M4a, LOW);
      analogWrite(M4b, pidController(PWM - PWM4Value));
    }
    if (PWM = 0) {
      digitalWrite(M4a, LOW);
      digitalWrite(M4b, LOW);
    }
    int PWM4Value = pidController(PWM - PWM4Value);
  }
}

float pidController(float error) {
  float kp = 0.1;
  float ki = 0;
  float kd = 0;
  static float previousError = 0.0;
  static float integral = 0.0;

  // Calculate proportional term
  float proportional = kp * error;

  // Calculate integral term
  integral += error;
  float integralTerm = ki * integral;

  // Calculate derivative term
  float derivative = kd * (error - previousError);

  // Update previous error
  previousError = error;

  // Calculate PID output
  float pidOut = proportional + integralTerm + derivative;

  // Clamp PID output to the range [-1, 1]
  if (pidOut > 1) pidOut = 1;
  if (pidOut < -1) pidOut = -1;

  pidOut = pidOut * 255;
  pidOut = (int)(pidOut);

  return pidOut;
}

void drive(float direction_deg, float speed, float rotation) {
    // Convert direction to radians
    float direction_rad = direction_deg * 3.14159265 / 180.0;
    float vx = speed * cos(direction_rad);
    float vy = speed * sin(direction_rad);

    // X-drive kinematics for 4 wheels: FL, FR, BR, BL
    float wheel_speeds[4];
    wheel_speeds[0] = 255 * vx * sin(45 * 3.14159265 / 180.0) + vy * cos(45 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[1] = 255 * vx * sin(-45 * 3.14159265 / 180.0) + vy * cos(-45 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[2] = 255 * vx * sin(-135 * 3.14159265 / 180.0) + vy * cos(-135 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[3] = 255 * vx * sin(135 * 3.14159265 / 180.0) + vy * cos(135 * 3.14159265 / 180.0) + rotation;

    // Normalize speeds if needed
    float max_speed = 0;
    for (int i = 0; i < 4; i++) {
        if (abs(wheel_speeds[i]) > max_speed) {
            max_speed = abs(wheel_speeds[i]);
        }
    }
    if (max_speed > 1.0) {
        for (int i = 0; i < 4; i++) {
            wheel_speeds[i] /= max_speed;
        }
    }

    SetSpeed(1, wheel_speeds[0]);
    SetSpeed(2, wheel_speeds[1]);
    SetSpeed(3, wheel_speeds[3]);
    SetSpeed(4, wheel_speeds[2]);
}

void SetSpeed(int Motor, int PWM) {
  static int PWM1Value = 0;
  static int PWM2Value = 0;
  static int PWM3Value = 0;
  static int PWM4Value = 0;

  if (Motor == 1) {
    if (PWM > 0) {
      analogWrite(M1a, PWM);
      digitalWrite(M1b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M1a, LOW);
      analogWrite(M1b, PWM);
    }
    if (PWM = 0) {
      digitalWrite(M1a, LOW);
      digitalWrite(M1b, LOW);
    }
    int PWM1Value = PWM;
  }

  if (Motor == 2) {
    if (PWM > 0) {
      analogWrite(M2a, PWM);
      digitalWrite(M2b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M2a, LOW);
      analogWrite(M2b, PWM);
    }
    if (PWM = 0) {
      digitalWrite(M2a, LOW);
      digitalWrite(M2b, LOW);
    }
    int PWM2Value = PWM;
  }

  if (Motor == 3) {
    if (PWM > 0) {
      analogWrite(M3a, PWM);
      digitalWrite(M3b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M3a, LOW);
      analogWrite(M3b, PWM);
    }
    if (PWM = 0) {
      digitalWrite(M3a, LOW);
      digitalWrite(M3b, LOW);
    }
    int PWM3Value = PWM;
  }

  if (Motor == 4) {
    if (PWM > 0) {
      analogWrite(M4a, PWM);
      digitalWrite(M4b, LOW);
    }
    if (PWM < 0) {
      digitalWrite(M4a, LOW);
      analogWrite(M4b, PWM);
    }
    if (PWM = 0) {
      digitalWrite(M4a, LOW);
      digitalWrite(M4b, LOW);
    }
    int PWM4Value = PWM;
  }
}