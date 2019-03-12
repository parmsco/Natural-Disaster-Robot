
#include <btsmotordriver.h>
void setup() {
BTS7960MotorDriver motor(7,8,9,10);
motor.Ready();
motor.TurnRight(255);
delay(500);
motor.Stop();
motor.Ready();
delay(500);
motor.TurnLeft(255);
delay(2500);
motor.Stop();
}


void loop() {

}
