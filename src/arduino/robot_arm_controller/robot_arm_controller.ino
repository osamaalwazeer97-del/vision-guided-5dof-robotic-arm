/*
  robot_arm_controller.ino

  Arduino Mega low-level controller for the 5-DOF vision-guided sorting arm.

  Extracted and cleaned from Appendix B of the graduation project report.
  It performs:
    - Encoder reading for base, shoulder, and elbow joints.
    - PID control of the three DC-motor joints.
    - Ramped target generation to reduce sudden motion.
    - Limit-switch based homing/reset behavior.
    - Servo control for pitch, roll, and gripper.
    - I2C communication with MATLAB.

  Required Arduino libraries:
    - PinChangeInterrupt
    - digitalPinFast
    - Servo
    - Wire

  Safety:
    - Verify pin mapping before connecting motors.
    - Test first with motor power disabled or low-current supply.
    - Confirm limit-switch directions before running homing.
*/

#include "PinChangeInterrupt.h"
#include <digitalPinFast.h>
#include <util/atomic.h>
#include <Wire.h>
#include <Servo.h>

// Encoder pins
#define CHA1 2   // external interrupt support
#define CHA2 22
#define CHB1 3   // external interrupt support
#define CHB2 23
#define CHC1 18  // external interrupt support
#define CHC2 24

// Encoder fast I/O objects
digitalPinFast EncoderA1(CHA1);
digitalPinFast EncoderB1(CHB1);
digitalPinFast EncoderC1(CHC1);
digitalPinFast EncoderA2(CHA2);
digitalPinFast EncoderB2(CHB2);
digitalPinFast EncoderC2(CHC2);

const int NumPosAng = 3;

volatile int posi[NumPosAng] = {0, 0, 0};
volatile int targetPos[NumPosAng] = {0, 0, 0};
int pos[NumPosAng];
int posprev[NumPosAng];
const int RANGE[NumPosAng] = {500, 400, 400};
volatile int PosdesiredAng[NumPosAng] = {0, 0, 0};
volatile int EN[NumPosAng] = {0, 0, 0};

// DC motor pins
const int pwm[] = {8, 9, 10};
const int in1[] = {34, 36, 38};
const int in2[] = {35, 37, 39};

// Limit switches
#define LSWA 19
#define LSWB 50
#define LSWC 51

volatile int oneshot[] = {0, 0, 0};
byte DontWork = 0;

// Servo and potentiometer pins
#define RollPot A0
#define PitPot1 A1
#define PitPot2 A2

Servo PitServo1;
Servo PitServo2;
Servo RollServo;
Servo GripServo;

const int PitInitAng  = 118;
const int RollInitAng = 60;
const int GripInitAng = 100;

int PitActAng1 = 0;
int PitActAng2 = 0;
int PitDesiredAng1 = 0;
int PitDesiredAng2 = PitInitAng;
int PitTarget1 = 0;
int PitTarget2 = 0;

int RollActAng = 0;
int RollDesiredAng = RollInitAng;
int RollTarget = 0;

int GripInit = 90;
int GripDesiredAng = GripInit;

const int indexGrip = 5;
const int index1 = 6;
const int index2 = 7;

int InitEN = 1;
int SerEN = 1;
int PosAchive = 0;
int error = 0;
int errors[] = {0, 0, 0};
const int tolerance[] = {5, 10, 10};
int shPwr = 0;

class SimplePID {
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;

  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(100), eprev(0.0), eintegral(0.0) {}

    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn;
      kd = kdIn;
      ki = kiIn;
      umax = umaxIn;
    }

    void evalu(int value, int target, int &e, float deltaT, int &pwr, int &dir, float &dedt, int en) {
      e = target - value;

      // Control of the initial value of the PID integral term
      if (en == 1) {
        eintegral = 0;
      } else if (en == -1) {
        eintegral = 50;
      } else if (en == 2) {
        eintegral = -50;
      }

      dedt = (float)(e - eprev) / deltaT;
      eintegral = eintegral + e * deltaT;

      float u = kp * e + kd * dedt + ki * eintegral;

      pwr = (int)fabs(u);
      if (pwr > umax) {
        pwr = umax;
      }

      dir = 1;
      if (u < 0) {
        dir = -1;
      }

      eprev = e;
    }
};

long prevT = 0;

// Receive matrix from MATLAB
int reciveMatrix[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t* vp = (uint8_t*) reciveMatrix;

SimplePID pid[NumPosAng];

void setup() {
  Serial.begin(9600);

  // Encoder setup
  EncoderA1.pinModeFast(INPUT_PULLUP);
  EncoderA2.pinModeFast(INPUT_PULLUP);
  EncoderB1.pinModeFast(INPUT_PULLUP);
  EncoderB2.pinModeFast(INPUT_PULLUP);
  EncoderC1.pinModeFast(INPUT_PULLUP);
  EncoderC2.pinModeFast(INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(CHA1), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(CHB1), readEncoderB, RISING);
  attachInterrupt(digitalPinToInterrupt(CHC1), readEncoderC, RISING);

  // DC motor setup
  for (int k = 0; k < NumPosAng; k++) {
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
  }

  // PID gains
  pid[0].setParams(4.5, 0.2, 1.0, 150.0);
  pid[1].setParams(3.0, 0.3, 0.8, 200.0);
  pid[2].setParams(6.0, 0.25, 1.0, 230.0);

  // Limit switch setup
  pinMode(LSWA, INPUT);
  pinMode(LSWB, INPUT);
  pinMode(LSWC, INPUT);

  attachInterrupt(digitalPinToInterrupt(LSWA), interruptLswA, FALLING);
  attachPCINT(digitalPinToPCINT(LSWB), interruptLswB, FALLING);
  attachPCINT(digitalPinToPCINT(LSWC), interruptLswC, FALLING);

  // Servo setup
  PitServo1.attach(4);
  PitServo2.attach(5);
  RollServo.attach(6);
  GripServo.attach(7);

  // I2C setup
  Wire.begin(8);                 // Join I2C bus with address #8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("target pos");

  if ((digitalRead(LSWA) == 0) || (digitalRead(LSWB) == 0) || (digitalRead(LSWC) == 0)) {
    DontWork = 1;
  } else {
    DontWork = 0;
  }

  // Put DC motor outputs initially to zero.
  // PWM is inverted because of the optocoupler stage.
  analogWrite(pwm[0], 255);
  analogWrite(pwm[1], 255);
  analogWrite(pwm[2], 255);
}

void loop() {
  // Reset orientation angle to the initial position
  if (InitEN == 1) {
    jointInitialOr();
  }

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  // reciveMatrix[index2] resets the DC motor positions to initial positions.
  if (reciveMatrix[index2] == 1) {
    jointInitialAng();
  }

  // Atomic read of encoder positions
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (int k = 0; k < NumPosAng; k++) {
      posprev[k] = pos[k];
      pos[k] = posi[k];
    }
  }

  for (int k = 0; k < NumPosAng; k++) {
    float Dpdt[NumPosAng];
    Dpdt[k] = ((float)(pos[k] - posprev[k]) / deltaT);

    // Ramped target to reduce sudden motion
    if (PosdesiredAng[k] - targetPos[k] > 2) {
      targetPos[k] = targetPos[k] + 1;
    } else if (PosdesiredAng[k] - targetPos[k] < -2) {
      targetPos[k] = targetPos[k] - 1;
    } else {
      targetPos[k] = PosdesiredAng[k];
    }

    int pwr, dir;
    float Dedt;

    // If power variable is off, hold current position and reset integral term.
    if (reciveMatrix[index1] == 0) {
      EN[k] = 1;
      PosdesiredAng[k] = posi[k];
      targetPos[k] = PosdesiredAng[k];
    }

    pid[k].evalu(pos[k], targetPos[k], error, deltaT, pwr, dir, Dedt, EN[k]);
    EN[k] = 0;
    errors[k] = PosdesiredAng[k] - pos[k];

    if ((reciveMatrix[index1] == 0) || DontWork == 1) {
      pwr = 0;
    }

    setMotor(dir, pwr, pwm[k], in1[k], in2[k]);

    Serial.print((int)(targetPos[k] * 0.4));
    Serial.print(" ");
    Serial.print((int)(pos[k] * 0.4));
    Serial.print(" ");
  }

  Serial.println();
  delay(30);

  // Check if desired position is achieved
  if (abs(errors[0]) < tolerance[0] &&
      abs(errors[1]) < tolerance[1] &&
      abs(errors[2]) < tolerance[2]) {
    if (SerEN == 1) {
      servoLoop();
    }
  } else {
    PosAchive = 0;
  }
}

void setMotor(int dir, int pwmVal, int pwmPin, int in1Pin, int in2Pin) {
  pwmVal = 255 - pwmVal;  // Invert PWM output because of optocoupler output
  analogWrite(pwmPin, pwmVal);

  if (dir == 1) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (dir == -1) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
  }
}

void receiveEvent(int howMany) {
  uint8_t* localVp = (uint8_t*) reciveMatrix;
  while (Wire.available()) {
    *localVp++ = Wire.read();
  }

  for (int k = 0; k < NumPosAng; k++) {
    targetPos[k] = pos[k];

    if (PosdesiredAng[k] != reciveMatrix[k]) {
      EN[k] = 1;
    } else {
      EN[k] = 0;
    }

    PosdesiredAng[k] = (int)(reciveMatrix[k] * 2.5); // Convert degrees to encoder slots
    errors[k] = PosdesiredAng[k] - pos[k];
  }

  if (pos[1] < -20) {
    setMotor(1, 20, pwm[1], in1[1], in2[1]);
    setMotor(1, 20, pwm[2], in1[2], in2[2]);
    EN[1] = -1;
    EN[2] = -1;
  }

  PitTarget2 = PitDesiredAng2;
  RollTarget = RollDesiredAng;
  PitDesiredAng2 = reciveMatrix[0 + NumPosAng] + PitInitAng;
  RollDesiredAng = reciveMatrix[1 + NumPosAng] + RollInitAng;
  GripDesiredAng = reciveMatrix[indexGrip];
  SerEN = 1;
}

void requestEvent() {
  jearkCancel();
  Wire.write(PosAchive);
  jearkCancel();
}

// Encoder interrupt service routines
void readEncoderA() {
  if (EncoderA2.digitalReadFast() > 0) {
    posi[0]++;
  } else {
    posi[0]--;
  }
}

void readEncoderB() {
  if (EncoderB2.digitalReadFast() > 0) {
    posi[1]++;
  } else {
    posi[1]--;
  }
}

void readEncoderC() {
  if (EncoderC2.digitalReadFast() > 0) {
    posi[2]++;
  } else {
    posi[2]--;
  }
}

void jointInitialAng() {
  PosdesiredAng[0] = pos[0] + RANGE[0];
  PosdesiredAng[1] = pos[1] + RANGE[1];
  PosdesiredAng[2] = pos[2] - RANGE[2];

  for (int k = 0; k < NumPosAng; k++) {
    EN[k] = 1;
  }

  reciveMatrix[index2] = 0;
}

void interruptLswA() {
  posi[0] = 225;     // 90 deg
  PosdesiredAng[0] = 0;
  targetPos[0] = posi[0];
  EN[0] = 1;
}

void interruptLswB() {
  cli();
  posi[1] = 100;     // 40 deg
  PosdesiredAng[1] = 0;
  targetPos[1] = posi[1] - 10;
  jearkCancelLSC();
  sei();
}

void interruptLswC() {
  cli();
  posi[2] = -32;     // -13 deg
  PosdesiredAng[2] = 50;
  targetPos[2] = posi[2] + 5;
  jearkCancelLSC();
  sei();
}

void readPot() {
  /*
    PitActAng1 = (int)((analogRead(PitPot1) - 58) / 3);
    PitActAng2 = (int)((analogRead(PitPot2) - 58) / 3);
    RollActAng = (int)((analogRead(RollPot) - 58) / 3);
    PitTarget1 = PitActAng1;
    PitTarget2 = PitActAng2;
    RollTarget = RollActAng;
  */
}

void servoLoop() {
  // readPot();

  // Drive gripper servo
  if ((GripDesiredAng >= 90) && (GripDesiredAng <= 160)) {
    if (GripDesiredAng > GripInit) {
      for (; GripInit <= GripDesiredAng; GripInit++) {
        jearkCancel();
        GripServo.write(GripInit);
        delay(5);
      }
    } else if (GripDesiredAng < GripInit) {
      for (; GripInit >= GripDesiredAng; GripInit--) {
        jearkCancel();
        GripServo.write(GripInit);
        delay(5);
      }
    } else {
      GripServo.write(GripInit);
    }
  }

  // Drive pitch servos
  if ((PitDesiredAng2 >= 5) && (PitDesiredAng2 <= 173)) {
    if (PitDesiredAng2 - PitTarget2 > 5) {
      for (; PitTarget2 <= PitDesiredAng2; PitTarget2++) {
        jearkCancel();
        PitTarget1 = 165 - PitTarget2;
        PitServo2.write(PitTarget2);
        PitServo1.write(PitTarget1);
        delay(30);
      }
    } else if (PitDesiredAng2 - PitTarget2 < -5) {
      for (; PitTarget2 >= PitDesiredAng2; PitTarget2--) {
        jearkCancel();
        PitTarget1 = 165 - PitTarget2;
        PitServo2.write(PitTarget2);
        PitServo1.write(PitTarget1);
        delay(30);
      }
    } else {
      PitServo2.write(PitDesiredAng2);
      PitServo1.write(165 - PitDesiredAng2);
    }
  }

  // Drive roll servo
  if ((RollDesiredAng >= 0) && (RollDesiredAng <= 170)) {
    if (RollDesiredAng - RollTarget > 5) {
      for (; RollTarget <= RollDesiredAng; RollTarget++) {
        jearkCancel();
        RollServo.write(RollTarget);
        delay(5);
      }
    } else if (RollDesiredAng - RollTarget < -5) {
      for (; RollTarget >= RollDesiredAng; RollTarget--) {
        jearkCancel();
        RollServo.write(RollTarget);
        delay(5);
      }
    } else {
      RollServo.write(RollDesiredAng);
    }
  }

  PosAchive = 1;
  SerEN = 0;
}

void jointInitialOr() {
  InitEN = 0;
  PitTarget2 = PitInitAng;
  PitDesiredAng2 = PitInitAng;
  RollTarget = RollInitAng;
  RollDesiredAng = RollInitAng;
  servoLoop();
}

void jearkCancel() {
  if (posi[1] < -40) {
    setMotor(1, 50, pwm[1], in1[1], in2[1]);
    setMotor(1, 50, pwm[2], in1[2], in2[2]);
    EN[1] = -1;
    EN[2] = -1;
  } else if (posi[1] > -40 && posi[1] < -20) {
    setMotor(1, 20, pwm[1], in1[1], in2[1]);
    setMotor(1, 20, pwm[2], in1[2], in2[2]);
    EN[1] = 1;
    EN[2] = 1;
  } else if (posi[1] > 20 && posi[1] < 40) {
    setMotor(-1, 20, pwm[1], in1[1], in2[1]);
    EN[1] = 1;
  } else if (posi[1] > 40) {
    setMotor(-1, 40, pwm[1], in1[1], in2[1]);
    EN[1] = 2;
  } else {
    EN[1] = 1;
    EN[2] = 1;
  }
}

void jearkCancelLSB() {
  setMotor(-1, 40, pwm[1], in1[1], in2[1]);
  EN[1] = 2;
}

void jearkCancelLSC() {
  setMotor(1, 40, pwm[2], in1[2], in2[2]);
  EN[2] = -1;
}
