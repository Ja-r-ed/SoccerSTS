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
}

void loop() {
  if (IrReceiver.decode()) {
    //if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
    //  IrReceiver.printIRResultRawFormatted(&Serial, true);
    //  IrReceiver.resume();  // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
    //} else {
      IrReceiver.resume();  // Early enable receiving of the next IR frame
      lastSignalTime = millis();
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
    //}
  }

  if (millis() - lastSignalTime > TIMEOUT_MS) {
    // No IR signal received recently — stop main code
    digitalWrite(M1a, LOW);
    digitalWrite(M1b, LOW);
    digitalWrite(M2a, LOW);
    digitalWrite(M2b, LOW);
    digitalWrite(M3a, LOW);
    digitalWrite(M3b, LOW);
    digitalWrite(M4a, LOW);
    digitalWrite(M4b, LOW);

    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);
    digitalWrite(dena, LOW);
  } else {
    // IR signal received recently — run main code
    if (IrReceiver.decodedIRData.command == 0x18) {  // forwards
      analogWrite(M1a, StraightPWM);
      digitalWrite(M1b, LOW);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, StraightPWM);

      analogWrite(M3a, StraightPWM);
      digitalWrite(M3b, LOW);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, StraightPWM);
    } else if (IrReceiver.decodedIRData.command == 0x52) {  // backwards
      analogWrite(M2a, StraightPWM);
      digitalWrite(M2b, LOW);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, StraightPWM);

      analogWrite(M4a, StraightPWM);
      digitalWrite(M4b, LOW);

      digitalWrite(M1a, LOW);
      analogWrite(M1b, StraightPWM);
    } else if (IrReceiver.decodedIRData.command == 0x5A) {  // right
      analogWrite(M1a, StraightPWM);
      digitalWrite(M1b, LOW);

      analogWrite(M2a, StraightPWM);
      digitalWrite(M2b, LOW);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, StraightPWM);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, StraightPWM);
    } else if (IrReceiver.decodedIRData.command == 0x8) {  // left
      digitalWrite(M1a, LOW);
      analogWrite(M1b, StraightPWM);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, StraightPWM);

      analogWrite(M3a, StraightPWM);
      digitalWrite(M3b, LOW);

      analogWrite(M4a, StraightPWM);
      digitalWrite(M4b, LOW);
    } else if (IrReceiver.decodedIRData.command == 0x45) {  // rotate left
      digitalWrite(M1a, LOW);
      analogWrite(M1b, RotatePWM);

      digitalWrite(M2a, LOW);
      analogWrite(M2b, RotatePWM);

      digitalWrite(M3a, LOW);
      analogWrite(M3b, RotatePWM);

      digitalWrite(M4a, LOW);
      analogWrite(M4b, RotatePWM);
    } else if (IrReceiver.decodedIRData.command == 0x46) { // dribble
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
    analogWrite(dena, 90);
    } else if (IrReceiver.decodedIRData.command == 0x47) {  // rotate right
      digitalWrite(M1b, LOW);
      analogWrite(M1a, RotatePWM);

      digitalWrite(M2b, LOW);
      analogWrite(M2a, RotatePWM);

      digitalWrite(M3b, LOW);
      analogWrite(M3a, RotatePWM);

      digitalWrite(M4b, LOW);
      analogWrite(M4a, RotatePWM);
    }
  }
}