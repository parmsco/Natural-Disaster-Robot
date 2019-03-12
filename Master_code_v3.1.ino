#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>
#include <btsmotordriver.h>
#include "Adafruit_VL53L0X.h"

BTS7960MotorDriver motorR(7, 8, 9, 10);
BTS7960MotorDriver motorL(2, 4, 5, 6);
VL53L0X sensor;
#define TCAADDR 0x70
MedianFilter test(10, 0);

extern "C"
{
#include "utility/twi.h"
}

//#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY

int Distance0, Distance1, Distance2, Distance3, Distance4, Distance5;

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

  sensor.init();
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(20000);
  Serial.println("Begin Prelimary Data Collection");
  for (int i = 0; i < 10; i++) {
    getDistance();
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  getDistance();
  if (Distance1 < 250 | Distance2 < 250 | Distance3 < 250 | Distance4 < 250) {
    motorsOFF();
    delay(250);
    turnRight(150);
    delay(1250);
    motorsOFF();

    getDistance();
    while (Distance0 < 350) {
      getDistance();
      if (Distance1 < 250 | Distance2 < 250 | Distance3 < 250 | Distance4 < 250) {
        motorsOFF();
      }
      else {
        forward(100);
      }
    }
    delay(500);
    motorsOFF();
    turnLeft(150);
    delay(1250);
    motorsOFF();
  } else {
    forward(100);
  }
}
void getDistance() {

  // sensor 0
  tcaselect(0);
  int val = sensor.readRangeSingleMillimeters();
  Distance0 = val;

  Serial.print("Sensor 0:  ");
  Serial.print(Distance0);
  Serial.print("   ");

  // sensor 1
  tcaselect(1);
  val = sensor.readRangeSingleMillimeters();
  Distance1 = val;

  Serial.print("Sensor 1:  ");
  Serial.print(Distance1);
  Serial.print("   ");

  // sensor 2
  tcaselect(2);
  val = sensor.readRangeSingleMillimeters();
  Distance2 = val;

  Serial.print("Sensor 2:  ");
  Serial.print(Distance2);
  Serial.print("   ");

  // sensor 3
  tcaselect(3);
  val = sensor.readRangeSingleMillimeters();
  Distance3 = val;

  Serial.print("Sensor 3:  ");
  Serial.print(Distance3);
  Serial.print("   ");

  // sensor 4
  tcaselect(4);
  val = sensor.readRangeSingleMillimeters();
  test.in(val);
  val = test.out();
  Distance4 = val;

  Serial.print("Sensor 4:  ");
  Serial.print(Distance4);
  Serial.println("   ");
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


