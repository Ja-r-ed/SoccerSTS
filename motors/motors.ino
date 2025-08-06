
#include <Arduino.h>


int M1a = 3;
int M1b = 4;
int M2a = 5;
int M2b = 6;
int M3a = 12;
int M3b = 13;
int M4a = 7;
int M4b = 11;

int d1 = 22;
int d2 = 23;
int dena = 2;

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
int M1Target = 0;
int M2Target = 0;
int M3Target = 0;
int M4Target = 0;

// Buffer for incoming serial data
float serialFloats[4] = {0, 0, 0, 0};

void setup() {
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

  // Open Serial1 for communication on RX1/TX1 (pins 19/18 on Mega)
  Serial1.begin(9600);
}

void loop() {
  // Read 4 floats from Serial1 if available
  if (Serial1.available() >= sizeof(float) * 4) {
    Serial1.readBytes((char*)serialFloats, sizeof(float) * 4);
    drive(serialFloats[0], serialFloats[1], serialFloats[2]);
    if (serialFloats[3] > 0.5) {
      dribble();
    }
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
    if (PWM == 0) {
      digitalWrite(M1a, LOW);
      digitalWrite(M1b, LOW);
    }
    PWM1Value = PWM;
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
    if (PWM == 0) {
      digitalWrite(M2a, LOW);
      digitalWrite(M2b, LOW);
    }
    PWM2Value = PWM;
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
    if (PWM == 0) {
      digitalWrite(M3a, LOW);
      digitalWrite(M3b, LOW);
    }
    PWM3Value = PWM;
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
    if (PWM == 0) {
      digitalWrite(M4a, LOW);
      digitalWrite(M4b, LOW);
    }
    PWM4Value = PWM;
  }
}
