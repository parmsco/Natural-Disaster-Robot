
#include <btsmotordriver.h>
#include "Adafruit_VL53L0X.h"

BTS7960MotorDriver motorR(7, 8, 9, 10);
BTS7960MotorDriver motorL(2, 4, 5, 6);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  // power
  Serial.println(F("VL53L0X API Simple Ranging example\n\n"));

}

void loop() {
  // put your main code here, to run repeatedly:

  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);
  int distance = measure.RangeMilliMeter;
  Serial.println(distance);

  forward(150);
  if (distance < 300) {
    motorsOFF();
  }
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
