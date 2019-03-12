
#include <btsmotordriver.h>
#include <EnableInterrupt.h>


int RC1;
int RC2;
int mode;
BTS7960MotorDriver motorR(7, 8, 9, 10);
BTS7960MotorDriver motorL(2, 4, 5, 6);


#define RC_NUM_CHANNELS  2

#define RC_CH1  0
#define RC_CH2  1

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1

void calc_ch1() {
  calc_input(RC_CH1, RC_CH1_INPUT);
}
void calc_ch2() {
  calc_input(RC_CH2, RC_CH2_INPUT);
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

void setup() {
  pinMode(12, INPUT);
  pinMode(3, INPUT);

  Serial.begin(57600);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
}
void loop() {
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
  if ( RC2 < 1600 && RC2 > 1400) { 
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
void activateGPSMode() {
  motorsOFF();
}
void  activateRCMode() {

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

