#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>
#include <btsmotordriver.h>
#include <EnableInterrupt.h>
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
int RC1;
int RC2;
int mode;


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
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(20000);

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

  // setup for RC
  pinMode(12, INPUT);
  pinMode(3, INPUT);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);

}
void getMode() {
  rc_read_values();
  mode = rc_values[RC_CH6];
}

void loop() {
  // put your main code here, to run repeatedly:
  getMode();
  if (mode < 1250) {
    motorsOFF();
  }
  else if ( mode > 1250 && mode < 1750) {
    RCmode();
  }
  else if (mode > 1750) {
   GPSmode();
  }
}
void GPSmode() {
  getDistance();
  if (Distance1 < 250 | Distance2 < 250 | Distance3 < 250 | Distance4 < 250) {
    avoidRight();
  } else {
    forward(100);
  }
}

void avoidRight(){
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
}
void RCmode() {
  rc_read_values();
  RC1 = rc_values[RC_CH1];
  RC2 = rc_values[RC_CH2];

  float timing = 25000;

  Serial.print("RC1: ");
  Serial.println(RC1);
  Serial.print("RC2: ");
  Serial.println(RC2);

  int val1 = abs(RC1 - 1500);
  int power1 = val1 / 2;
  int val2 = abs(RC2 - 1500);
  int power2 = val2 / 2;

  if (RC1 == 0 | RC2 == 0 ) {
    motorsOFF();
  }
  if (RC1 < 1600 && RC1 > 1400 ) {
    motorsOFF();
  }
  if ( RC2 < 1600 && RC2 < 1400) {
    motorsOFF();
  }
  if (RC1 > 1600) {
    turnRight(power1);
  }
  else if (RC1 < 1400 && RC1 > 900) {
    turnLeft(power1);
  }
  else if (RC2 < 1400 && RC2 > 900) {
    backward(power2);
  }
  else if (RC2 > 1600) {
    forward(power2);
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
