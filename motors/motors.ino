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
      analogWrite(M1a, 40);
      digitalWrite(M1b, LOW);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, 40);

      analogWrite(M3a, 40);
      digitalWrite(M3b, LOW);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, 40);
    } else if (IrReceiver.decodedIRData.command == 0x52) {  // backwards
      analogWrite(M2a, 40);
      digitalWrite(M2b, LOW);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, 40);

      analogWrite(M4a, 40);
      digitalWrite(M4b, LOW);

      digitalWrite(M1a, LOW);
      analogWrite(M1b, 40);
    } else if (IrReceiver.decodedIRData.command == 0x5A) {  // turn right
      digitalWrite(M1a, LOW);
      analogWrite(M1b, 100);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, 100);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, 100);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, 100);
    } else if (IrReceiver.decodedIRData.command == 0x8) {  // turn left
      digitalWrite(M1a, LOW);
      analogWrite(M1b, 100);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, 100);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, 100);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, 100);
    } else if (IrReceiver.decodedIRData.command == 0x45) {  // turn left
      digitalWrite(M1a, LOW);
      analogWrite(M1b, 100);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, 100);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, 100);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, 100);
    } else if (IrReceiver.decodedIRData.command == 0x46) {  // dribble

    } else if (IrReceiver.decodedIRData.command == 0x47) {  // turn right
      digitalWrite(M1a, LOW);
      analogWrite(M1b, 100);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, 100);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, 100);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, 100);
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

void RobotDrive(float Speed, int Direction, float Rotation) {
  float pi = 3.1415926;
  float T = Direction * 3.14159265 / 180.0;  // Convert to radians
  float R = Rotation;
  float S = Speed;


  float P1 = -((cos(T + (pi / 4.0) * (-abs(R) + 1))) / (cos(pi / 4.0)));
  float P2 = -((cos(T + (3 * pi / 4.0) * (-abs(R) + 1))) / (cos(pi / 4.0)));

  float Pfl = P2 + R;
  float Pfr = P1 - R;
  float Pbl = P1 + R;
  float Pbr = P2 - R;

  float biggest = 0;
  if (abs(Pfl) > abs(Pfr)) {
    biggest = abs(Pfl);
  }
  if (abs(Pbl) > biggest) {
    biggest = abs(Pbl);
  }
  if (abs(Pbr) > biggest) {
    biggest = abs(Pbr);
  }

  float s = biggest / S;

  float Mfl = Pfl / s;
  float Mfr = Pfr / s;
  float Mbl = Pbl / s;
  float Mbr = Pbr / s;

  Mfl = Mfl * 255;
  Mfr = Mfr * 255;
  Mbl = Mbl * 255;
  Mbr = Mbr * 255;

  SetSpeed(1, Mfl);
  SetSpeed(2, Mfr);
  SetSpeed(3, Mbl);
  SetSpeed(4, Mbr);
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