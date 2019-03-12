#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>
#include <btsmotordriver.h>
#include <EnableInterrupt.h>
#include "Adafruit_VL53L0X.h"
#include <MechaQMC5883.h>

BTS7960MotorDriver motorR(7, 8, 9, 10);
BTS7960MotorDriver motorL(2, 4, 5, 6);
VL53L0X sensor;
MechaQMC5883 qmc;
#define TCAADDR 0x70
MedianFilter test(10, 0);

extern "C"
{
#include "utility/twi.h"
}

//#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY

int distLeft, distRight, distFront, distFrontRight, distFrontLeft;
int RC1;
int RC2;
int mode;
int x, y, z;
int azimuth;
int heading;


#define RC_NUM_CHANNELS  3

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH6  2

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH6_INPUT  A2

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_INPUT);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_INPUT);
}
void calc_ch6() {
  calc_input(RC_CH6, RC_CH6_INPUT);
}

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Begin Test");

  // sensor setup and pause to ensure we are geting values
  sensor.init();
  qmc.init();
  sensor.setTimeout(500);

  //frequency of how we take in data
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  pinMode(13, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);
  getCompass();
  heading = azimuth;

  getDistance();
  while (distLeft < 1 | distRight < 1 | distFront < 1 | distFrontRight < 1 | distFrontLeft < 1) {
    setup();
    getDistance();
  }

}
void getMode() {
  rc_read_values();
  mode = rc_values[RC_CH6];
}
/*
  void turnRight90() {
  motorsOFF();
  delay(250);
  turnRight(150);
  delay(1000);
  motorsOFF();
  }
*/
void turnRight90() {
  int goal;
  if (azimuth > 270) {
    goal = 360 - azimuth;
  }
  else {
    goal = azimuth + 90;
  }
  while (azimuth  > goal +5 | azimuth < goal -5) {
    turnRight(150);
    getCompass();
  }
}

  void turnLeft90() {
  motorsOFF();
  delay(500);
  turnLeft(150);
  delay(1000);
  motorsOFF();
  }

void turnAround() {
  motorsOFF();
  delay(500);
  turnLeft(150);
  delay(2500);
  motorsOFF();
}
void loop() {
  // put your main code here, to run repeatedly:
  //NEED TO CHECK WIRING FOR WHICH SENSOR IS FROM WHICH PLACE
  //GETDISTANCES WILL NOT WORK UNTIL THIS IS CHECKED AND FIXED

  if (digitalRead(13) == HIGH) {
    Serial.println("TURNOFF");
    motorsOFF();
    delay(100);
  }
  getMode();
  if (mode < 1250) {
    motorsOFF();
  }
  else if ( mode > 1250 && mode < 1750) {
    RCmode();
  }
  else if (mode > 1750) {
    Serial.print("Azimuth: ");
    Serial.println(getCompass());
    //GPSmode();
    
  }
}
void GPSmode() {
  getDistance();
  if (distFront < 400 | distFrontRight < 400 | distFrontLeft < 400) {
    if (distLeft < 4000 && distRight < 4000) {
      //avoidDoubleCorner();
      motorsOFF();
    }
    else if (distFrontLeft > distFrontRight) {
      avoidLeft();
    }
    else {
      avoidRight();
    }
  }
  else {
    forward(100);
    getDistance();
  }
}
void avoidDoubleCorner() {
  while (distLeft < 350 && distRight < 350) {
    backward(100);
  }
  if (distLeft > distRight) {
    turnLeft90();
  }
  else {
    turnRight90();
  }
  forward(150);
  delay(500);
}

void avoidRight() {
  turnRight90();
  getDistance();
  while (distLeft < 350) {
    getDistance();
    if (distFront < 250 | distFrontRight < 250 | distFrontLeft < 250) {
      motorsOFF();
      //check software flowchart for what to do once rest fo code works
    }
    else {
      forward(100);
    }
  }
  forward(100);
  delay(1000);
  turnLeft90();
  motorsOFF();

}
void avoidLeft() {
  turnLeft90();
  getDistance();

  while (distRight < 350) {
    getDistance();
    if (distFront < 250 | distFrontRight < 250 | distFrontLeft < 250) {
      motorsOFF();
      //check software flowchart for what to do once rest of code works
    }
    else {
      forward(100);
    }
  }
  forward(100);
  delay(1000);
  turnRight90();
}

void RCmode() {
  rc_read_values();
  RC1 = rc_values[RC_CH1];
  RC2 = rc_values[RC_CH2];

  float timing = 25000;

  Serial.print("RC1: ");
  Serial.print(RC1);
  Serial.print("   ");
  Serial.print("RC2: ");
  Serial.println(RC2);

  int val1 = abs(RC1 - 1500);
  int power1 = val1 / 2;
  int val2 = abs(RC2 - 1500);
  int power2 = val2 / 2;

  if (RC1 == 0 | RC2 == 0 ) {
    motorsOFF();
  }
  if (RC1 < 1550 && RC1 > 1450 ) {
    motorsOFF();
  }
  if ( RC2 < 1550 && RC2 > 1450) {
    motorsOFF();
  }
  if (RC1 > 1550) {
    turnRight(power1);
  }
  else if (RC1 < 1450 && RC1 > 900) {
    turnLeft(power1);
  }
  else if (RC2 < 1450 && RC2 > 900) {
    backward(power2);
  }
  else if (RC2 > 1550) {
    forward(power2);
  }
}
int getCompass() {

  //float azimuth; //is supporting float too
  qmc.read(&x, &y, &z, &azimuth);
  //azimuth = qmc.azimuth(&y,&x);//you can get custom azimuth
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.print(z);
  Serial.print(" a: ");
  Serial.print(azimuth);
  Serial.println();
  return azimuth;
}
void getDistance() {

  // sensor 0
  tcaselect(0);
  int val = sensor.readRangeSingleMillimeters();
  distLeft = val;

  Serial.print("distLeft:  ");
  Serial.print(distLeft);
  Serial.print("   ");

  // sensor 1
  tcaselect(1);
  val = sensor.readRangeSingleMillimeters();
  distFrontLeft = val;

  Serial.print("distFrontLeft:  ");
  Serial.print(distFrontLeft);
  Serial.print("   ");


  // sensor 2
  tcaselect(2);
  val = sensor.readRangeSingleMillimeters();
  distFront = val;

  Serial.print("distFront:  ");
  Serial.print(distFront);
  Serial.print("   ");


  // sensor 3
  tcaselect(3);
  val = sensor.readRangeSingleMillimeters();
  distFrontRight = val;

  Serial.print("distFrontRight:  ");
  Serial.print(distFrontRight);
  Serial.print("   ");

  // sensor 4
  tcaselect(4);
  val = sensor.readRangeSingleMillimeters();
  test.in(val);
  val = test.out();

  distRight = val;
  Serial.print("distRight:  ");
  Serial.println(distRight);
  delay(2);
}

void forward(int power) {
  motorL.Ready();
  motorR.Ready();

  motorR.TurnRight(power);
  motorL.TurnLeft(power);
}
void backward(int power) {
  motorL.Ready();
  motorR.Ready();

  motorR.TurnLeft(power);
  motorL.TurnRight(power);
}

void turnLeft(int power) {
  motorL.Ready();
  motorR.Ready();

  motorR.TurnRight(power);
  motorL.TurnRight(power);
}
void turnRight(int power) {
  motorL.Ready();
  motorR.Ready();

  motorR.TurnLeft(power);
  motorL.TurnLeft(power);
}
void motorsOFF() {
  motorR.Ready();
  motorL.Ready();

  motorR.Stop();
  motorL.Stop();
}
