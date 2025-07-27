#include <IRremote.hpp>
#include <Arduino.h>
#define DECODE_NEC

int M1a = 5;
int M1b = 4;
int M2a = 3;
int M2b = 2;
int M3a = 6;
int M3b = 7;
int M4a = 12;
int M4b = 11;

int d1 = 22;
int d2 = 23;
int dena = 13;

/*
Front Left  (M1)
Front Right (M2)
Back Left   (M3)
Back Right  (M4)

m4a = YELLOW, 11 left side of robot
m4B = BLACK, 12
m2a = ORANGE, 2
m2b = BROWN, 3 
m1a = YELLOW, 4 right side of robot
m1b = ORANGE, 5
m3a = GREY, 6
m3b = WHITE, 7 

d1 = RED, 22
d2 = BLACK, 23
dena = BROWN, 13
*/

int RotatePWM = 50;
int StraightPWM = 60;

const unsigned long TIMEOUT_MS = 200;  // 3 seconds timeout
unsigned long lastSignalTime = 0;

int M1Target = 0;
int M2Target = 0;
int M3Target = 0;
int M4Target = 0;

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
  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(dena, OUTPUT);
  delay(100);
}

void loop() {
  // if (IrReceiver.decode()) {
  //   //if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
  //   //  IrReceiver.printIRResultRawFormatted(&Serial, true);
  //   //  IrReceiver.resume();  // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
  //   //} else {
  //     IrReceiver.resume();  // Early enable receiving of the next IR frame
  //     lastSignalTime = millis();
  //     IrReceiver.printIRResultShort(&Serial);
  //     IrReceiver.printIRSendUsage(&Serial);
  //   //}
  // }

  // if (millis() - lastSignalTime > TIMEOUT_MS) {
  //   // No IR signal received recently — stop main code
  //   digitalWrite(M1a, LOW);
  //   digitalWrite(M1b, LOW);
  //   digitalWrite(M2a, LOW);
  //   digitalWrite(M2b, LOW);
  //   digitalWrite(M3a, LOW);
  //   digitalWrite(M3b, LOW);
  //   digitalWrite(M4a, LOW);
  //   digitalWrite(M4b, LOW);

  //   digitalWrite(d1, LOW);
  //   digitalWrite(d2, LOW);
  //   digitalWrite(dena, LOW);
  // } else {
  //   // IR signal received recently — run main code
  //   if (IrReceiver.decodedIRData.command == 0x18) {  // forwards
  //     drive(0, 0.2, 0);
  //   } else if (IrReceiver.decodedIRData.command == 0x52) {  // backwards
  //     drive(180, 0.2, 0);
  //   } else if (IrReceiver.decodedIRData.command == 0x5A) {  // right
  //     drive(90, 0.2, 0);
  //   } else if (IrReceiver.decodedIRData.command == 0x8) {  // left
  //     drive(270, 0.2, 0);
  //   } else if (IrReceiver.decodedIRData.command == 0x45) {  // rotate left
  //     digitalWrite(M1a, LOW);
  //     analogWrite(M1b, RotatePWM);

  //     digitalWrite(M2a, LOW);
  //     analogWrite(M2b, RotatePWM);

  //     digitalWrite(M3a, LOW);
  //     analogWrite(M3b, RotatePWM);

  //     digitalWrite(M4a, LOW);
  //     analogWrite(M4b, RotatePWM);
  //   } else if (IrReceiver.decodedIRData.command == 0x46) { // dribble
  //   digitalWrite(d1, LOW);
  //   digitalWrite(d2, HIGH);
  //   analogWrite(dena, 90);
  //   } else if (IrReceiver.decodedIRData.command == 0x47) {  // rotate right
  //     digitalWrite(M1b, LOW);
  //     analogWrite(M1a, RotatePWM);

  //     digitalWrite(M2b, LOW);
  //     analogWrite(M2a, RotatePWM);

  //     digitalWrite(M3b, LOW);
  //     analogWrite(M3a, RotatePWM);

  //     digitalWrite(M4b, LOW);
  //     analogWrite(M4a, RotatePWM);
  //   }
  // }

  // drive(0, 0.2, 0);
  // delay(1000);

  // drive(90, 0.2, 0);
  // delay(1000);

  // drive(180, 0.2, 0);
  // delay(1000);

  // drive(270, 0.2, 0);
  // delay(1000);
  drive(0, 0.4, 0);
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
  digitalWrite(d1, LOW);
  digitalWrite(d2, HIGH);
  analogWrite(dena, 70);
}

void drive(float direction_deg, float speed, float rotation) {
    // Convert direction to radians
    float direction_rad = direction_deg * 3.14159265 / 180.0;
    float vx = speed * cos(direction_rad);
    float vy = speed * sin(direction_rad);

    // X-drive kinematics for 4 wheels: FL, FR, BR, BL
    float wheel_speeds[4];
    wheel_speeds[0] = vx * sin(45 * 3.14159265 / 180.0) + vy * cos(45 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[1] = vx * sin(-45 * 3.14159265 / 180.0) + vy * cos(-45 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[2] = vx * sin(-135 * 3.14159265 / 180.0) + vy * cos(-135 * 3.14159265 / 180.0) + rotation;
    wheel_speeds[3] = vx * sin(135 * 3.14159265 / 180.0) + vy * cos(135 * 3.14159265 / 180.0) + rotation;

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

    SetSpeed(1, wheel_speeds[0]*255);
    SetSpeed(2, wheel_speeds[1]*255);
    SetSpeed(3, wheel_speeds[3]*255);
    SetSpeed(4, wheel_speeds[2]*255);
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
      analogWrite(M1b, -PWM);
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
      analogWrite(M2b, -PWM);
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
      analogWrite(M3b, -PWM);
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
      analogWrite(M4b, -PWM);
    }
    if (PWM = 0) {
      digitalWrite(M4a, LOW);
      digitalWrite(M4b, LOW);
    }
    int PWM4Value = PWM;
  }
}