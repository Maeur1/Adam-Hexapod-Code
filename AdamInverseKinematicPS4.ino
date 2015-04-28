#include <PS4BT.h>
#include <usbhub.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

int SERVOMAX[] = {
  550, 510, 550, 550, 550, 530, 520, 520, 550, 550, 550, 535, 565, 500, 530, 530
}; // this is the 'maximum' pulse length count (out of 4096)
int SERVOMIN[] = {
  130, 135, 170, 190, 123, 140, 140, 130, 140, 135, 190, 130, 150, 140, 200, 150
}; // this is the 'minimum' pulse length count (out of 4096)

Servo s16;
Servo s17;
// our servo # counter
uint8_t servonum = 0;

double x = 0;
double y = 0;
double z = 0;
double rotX = 0;
double rotY = 0;
double rotZ = 0;
double hexagon = 85.0; //This is the length of each side of the hexagon the hexapod makes
double coxaLength = 26.0; //Length of the pivot from the robot to the first pivot on the leg
double femurLength = 49.0; //Length from the second pivot to the 3rd pivot point
double tibiaLength = 138.0; //Length from the last pivot point to the end of the leg
double BodyCoxaOffsetX = hexagon / 2;
double BodyCoxaOffsetZ = sqrt(hexagon * hexagon - BodyCoxaOffsetX * BodyCoxaOffsetX);
double BodyCoxaOffset = hexagon;
double legX[] = {
  BodyCoxaOffsetX, BodyCoxaOffset, BodyCoxaOffsetX, -BodyCoxaOffsetX, -BodyCoxaOffset, -BodyCoxaOffsetX
};
double legZ[] = {
  BodyCoxaOffsetZ, 0, -BodyCoxaOffsetZ, -BodyCoxaOffsetZ, 0, BodyCoxaOffsetZ
};
double tibiaAngle;
double femurAngle;
double coxaAngle;
double totalIZ;
double totalIX;
double DistBodyCenterFeet;
double AngleBodyCenterX;
double FeetPosX;
double FeetPosY;
double FeetPosZ;
double CoxaFeetDist;
double IKSW;
double IKA1;
double IKA2;
double TAngle;
double NewPosX;
double NewPosY;
double NewPosZ;
double IKTibiaAngle;
double IKFemurAngle;
double IKCoxaAngle;
double RollY;
double PitchY;
double BodyIkX;
double BodyIkZ;
double BodyIkY;
double servoAngle[] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
double servoTarget[18];
int times = 20;
const float pi = 3.14159265358979323;

void setup() {
  Serial.begin(115200);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
}

void loop() {
  Usb.Task();
  if (PS4.connected()) {
    if (PS4.getAnalogHat(RightHatX) > 137 || PS4.getAnalogHat(RightHatX) < 117 || PS4.getAnalogHat(RightHatY) > 137 || PS4.getAnalogHat(RightHatY) < 117) {
      x = map(PS4.getAnalogHat(RightHatX), 0, 255, 10, -10);
      z = map(PS4.getAnalogHat(RightHatY), 0, 255, 10, -10);
    }
    else if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117) {
      rotY  = map(PS4.getAnalogHat(LeftHatY), 0, 255, -15, 15);
      rotX = map(PS4.getAnalogHat(LeftHatX), 0, 255, -10, 10);
    }
    else if (PS4.getButtonClick(TRIANGLE)) {
      forwardStep();
    }
    else if (PS4.getButtonClick(CROSS)) {
      backwardStep();
    }
    else if (PS4.getButtonClick(SQUARE)) {
      strafeLeft();
    }
    else if (PS4.getButtonClick(CIRCLE)) {
      strafeRight();
    }
    else if (PS4.getAnalogButton(L2) > 0 || PS4.getAnalogButton(R2) > 0 ) {
      rotZ = map(PS4.getAnalogButton(L2) - PS4.getAnalogButton(R2), -255, 255, -10, 10);
    }
    else {
      x = 0;
      z = 0;
      rotZ = 0;
      rotY = 0;
      rotX = 0;
    }
    for (uint8_t i = 0; i < 2; i++) { // The touchpad track two fingers
      if (PS4.isTouching(i)) { // Print the position of the finger if it is touching the touchpad
        y = map(PS4.getX(i), 0, 1900, 30.0, -30.0);
      }
    }
    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS4.disconnect();
    }
    if (PS4.getButtonClick(UP)) {
        times += 10;
      } if (PS4.getButtonClick(DOWN)) {
        if(times > 0){
        times -= 10;
        }
      }
    for (int i = 0; i < 6; i++) {
      Legs(i);
    }
    refresh();
  }
  else {
    turnServosOff();
  }
}

boolean array_cmp(double *a, double *b) {
  int n;
  // test each element to be the same. if not, return false
  for (n = 0; n < 18; n++) if ((int)a[n] != (int)b[n]) return false;

  //ok, if we have not returned yet, they are equal :)
  return true;
}

void refresh() {
  while (!array_cmp(servoTarget, servoAngle)) {
    for (int i = 0; i < 18; i++) {
      if (servoAngle[i] > servoTarget[i]) {
        servoAngle[i]--;
        if (servoAngle[i] > servoTarget[i]) {
          servoAngle[i]--;
          if (servoAngle[i] > servoTarget[i]) {
            servoAngle[i]--;
          }
        }
      } else if (servoAngle[i] < servoTarget[i]) {
        servoAngle[i]++;
        if (servoAngle[i] < servoTarget[i]) {
          servoAngle[i]++;
          if (servoAngle[i] < servoTarget[i]) {
            servoAngle[i]++;
          }
        }
      }
      if (i < 16) {
        double pulselength = map(servoAngle[i], 0, 180, SERVOMIN[i], SERVOMAX[i]);
        pwm.setPWM(i, 0, pulselength);
      } else {
        if (i == 17) {
          s17.write(servoAngle[i] + 15);
        } else {
          s16.write(servoAngle[i]);
        }
      }
    }
    delay(times);
  }
}

void strafeRight() {
  double yStart = y;
  y = 0;
  T1();
  rotY = 15;
  T1();
  y = yStart;
  rotY = -15;
  T2();
  rotY = 15;
  T1();
  y = 0;
  rotY = -15;
  T2();
  rotY = 0;
  T2();
  y = yStart;
  rotY = 0;
  T1();
  rotY = 0;
  T2();
  rotY = 0;
  refresh();
}

void strafeLeft() {
  double yStart = y;
  y = 0;
  T1();
  rotY = -15;
  T1();
  y = yStart;
  rotY = 15;
  T2();
  rotY = -15;
  T1();
  y = 0;
  rotY = 15;
  T2();
  rotY = 0;
  T2();
  y = yStart;
  rotY = 0;
  T1();
  rotY = 0;
  T2();
  rotY = 0;
  refresh();
}

void forwardStep() {
  double yStart = y;
  y = 0;
  T1(); //T1 Upward
  rotY = 15;
  Legs(0);
  Legs(2);
  rotY = -15;
  Legs(4);
  refresh(); //T1 Forward
  y = yStart;
  rotY = 15;
  Legs(3);
  Legs(5);
  rotY = -15;
  Legs(1);
  refresh();// T2 Backward
  y = yStart;
  rotY = 15;
  Legs(0);
  Legs(2);
  rotY = -15;
  Legs(4);
  refresh(); //T1 Downward
  y = 0;
  rotY = 15;
  Legs(3);
  Legs(5);
  rotY = -15;
  Legs(1);
  refresh(); //T2 Upward
  rotY = 0;
  T2(); //T2 Forward
  y = yStart;
  rotY = 0;
  T1(); //T1 Backward
  T2(); //T2 Downward
  refresh();
}

void backwardStep() {
  double yStart = y;
  y = 0;
  T1(); //T1 Upward
  rotY = -15;
  Legs(0);
  Legs(2);
  rotY = 15;
  Legs(4);
  refresh(); //T1 Forward
  y = yStart;
  rotY = -15;
  Legs(3);
  Legs(5);
  rotY = 15;
  Legs(1);
  refresh();// T2 Backward
  y = yStart;
  rotY = -15;
  Legs(0);
  Legs(2);
  rotY = 15;
  Legs(4);
  refresh(); //T1 Downward
  y = 0;
  rotY = -15;
  Legs(3);
  Legs(5);
  rotY = 15;
  Legs(1);
  refresh(); //T2 Upward
  rotY = 0;
  T2(); //T2 Forward
  y = yStart;
  rotY = 0;
  T1(); //T1 Backward
  T2(); //T2 Downward
  refresh();
}

void T1() {
  Legs(0);
  Legs(2);
  Legs(4);
  refresh();
}

void T2() {
  Legs(1);
  Legs(3);
  Legs(5);
  refresh();
}

void Legs(int leg) {
  switch (leg) {
    case 0:
      FeetPosX = (cos(60.0 / 180.0 * 3.141592)) * (coxaLength + femurLength);
      FeetPosZ = sin(60.0 / 180.0 * 3.141592) * (coxaLength + femurLength);
      break;
    case 1:
      FeetPosX = (coxaLength + femurLength);
      FeetPosZ = 0;
      break;
    case 2:
      s16.attach(2);
      FeetPosX = (cos(60.0 / 180.0 * 3.141592)) * (coxaLength + femurLength);
      FeetPosZ = -sin(60.0 / 180.0 * 3.141592) * (coxaLength + femurLength);
      break;
    case 3:
      s17.attach(3);
      FeetPosX = -(cos(60.0 / 180.0 * 3.141592)) * (coxaLength + femurLength);
      FeetPosZ = -sin(60.0 / 180.0 * 3.141592) * (coxaLength + femurLength);
      break;
    case 4:
      FeetPosX = -(coxaLength + femurLength);
      FeetPosZ = 0;
      break;
    case 5:
      FeetPosX = -(cos(60.0 / 180.0 * 3.141592)) * (coxaLength + femurLength);
      FeetPosZ = sin(60.0 / 180.0 * 3.141592) * (coxaLength + femurLength);
      break;
  }
  FeetPosY = tibiaLength;
  totalIX = FeetPosX + legX[leg] + x;
  totalIZ = FeetPosZ + legZ[leg] + z;
  DistBodyCenterFeet = sqrt((totalIX * totalIX) + (totalIZ * totalIZ));
  AngleBodyCenterX = (pi / 2.0) - atan2(totalIX, totalIZ);
  RollY = tan(rotZ * pi / 180.0) * totalIX;
  PitchY = tan(rotX * pi / 180.0) * totalIZ;
  BodyIkX = cos(AngleBodyCenterX + (rotY * pi / 180.0)) * DistBodyCenterFeet - totalIX;
  BodyIkZ = (sin(AngleBodyCenterX + (rotY * pi / 180.0)) * DistBodyCenterFeet - totalIZ);
  BodyIkY = RollY + PitchY;
  NewPosX = FeetPosX + x + BodyIkX;
  NewPosY = FeetPosY + y + BodyIkY;
  NewPosZ = FeetPosZ + z + BodyIkZ;
  CoxaFeetDist = sqrt((NewPosX * NewPosX) + (NewPosZ * NewPosZ));
  IKSW = sqrt(((CoxaFeetDist - coxaLength) * (CoxaFeetDist - coxaLength)) + (NewPosY * NewPosY));
  IKA1 = atan2((CoxaFeetDist - coxaLength), NewPosY);
  IKA2 = acos((tibiaLength * tibiaLength - femurLength * femurLength - IKSW * IKSW) / (-2.0 * IKSW * femurLength));
  TAngle = acos((IKSW * IKSW - femurLength * femurLength - tibiaLength * tibiaLength) / (-2.0 * tibiaLength * femurLength));
  IKTibiaAngle = (90 - TAngle * 180.0 / pi);
  IKFemurAngle = (IKA1 + IKA2) * 180.0 / 3.14159265358979323 - 90;
  IKCoxaAngle = 90 - atan2(NewPosZ, NewPosX) * 180.0 / 3.14159265358979323;
  switch (leg) {
    case 0:
      moveLegCoxa(leg + 1, IKCoxaAngle + 60);
      break;
    case 1:
      moveLegCoxa(leg + 1, IKCoxaAngle);
      break;
    case 2:
      moveLegCoxa(leg + 1, IKCoxaAngle - 60);
      break;
    case 3:
      moveLegCoxa(leg + 1, IKCoxaAngle + 60 + 180 - 360);
      break;
    case 4:
      if (IKCoxaAngle > 180) {
        moveLegCoxa(leg + 1, IKCoxaAngle - 180);
        break;
      }
      else {
        moveLegCoxa(leg + 1, IKCoxaAngle + 180);
        break;
      }
    case 5:
      moveLegCoxa(leg + 1, IKCoxaAngle - 60 + 180);
      break;
  }
  moveLegTibia(leg + 1, IKTibiaAngle);
  moveLegFemur(leg + 1, IKFemurAngle);

}

/*First leg Shoe:  12
 First leg Quad:  11
 First leg Hip:   10
 Second leg Shoe: 15
 Second leg Quad: 14
 Second leg Hip:  13
 Thrid leg Shoe:  16
 Thrid leg Quad:  06
 Third leg Hip:   05
 Forth leg Shoe:  17
 Forth leg Quad:  04
 Forth leg Hip:   03
 Fifth leg Shoe:  00
 Fifth leg Quad:  01
 Fifth leg Hip:   02
 Sixth leg Shoe:  09
 Sixth leg Quad:  08
 Sixth leg Hip:   07
 */
void setServoDegrees(uint16_t n, int degree) {
  servoTarget[n] = degree;
}

void moveLegFemur(int leg, int n) {
  switch (leg) {
    case 1:
      setServoDegrees(11, 90 + n);
      break;
    case 2:
      setServoDegrees(14, 90 + n);
      break;
    case 3:
      setServoDegrees(6, 90 + n);
      break;
    case 4:
      setServoDegrees(4, 90 - n);
      break;
    case 5:
      setServoDegrees(1, 90 - n);
      break;
    case 6:
      setServoDegrees(8, 90 - n);
      break;
  }
}

void moveLegCoxa(int leg, int n) {
  switch (leg) {
    case 1:
      setServoDegrees(10, n);
      break;
    case 2:
      setServoDegrees(13, n);
      break;
    case 3:
      setServoDegrees(5, n);
      break;
    case 4:
      setServoDegrees(3, n);
      break;
    case 5:
      setServoDegrees(2, n);
      break;
    case 6:
      setServoDegrees(7, n);
      break;
  }
}

void moveLegTibia(int leg, int n) {
  switch (leg) {
    case 1:
      setServoDegrees(12, n);
      break;
    case 2:
      setServoDegrees(15, n);
      break;
    case 3:
      setServoDegrees(16, n);
      break;
    case 4:
      setServoDegrees(17, 180 - n);
      break;
    case 5:
      setServoDegrees(0, 180 - n);
      break;
    case 6:
      setServoDegrees(9, 180 - n);
      break;
  }
}

void turnServosOff() {
  s16.detach();
  s17.detach();
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 0);
  }
}


