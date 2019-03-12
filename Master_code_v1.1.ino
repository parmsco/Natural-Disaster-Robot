
#include <btsmotordriver.h>
void setup() {
  Serial.begin(9600);
}

void loop() {
  for(int i = 0; i< 50; i++){
     if(i < 25){
     forward(100);
     delay(250);
     }
     if(i==25){
      motorsOFF();
     }
     Serial.println(i);
     if(i > 25){
      backward(100);
      delay(250);
     }
     if(i==50){
      motorsOFF();
     }
  }
   
}
void activateGPSMode() {

}
void  activateRCMode() {

}
void forward(int power) {
  BTS7960MotorDriver motorR(7, 8, 9, 10);
  BTS7960MotorDriver motorL(2, 4, 5, 6);

  motorL.Ready();
  motorR.Ready();

  motorR.TurnRight(power);
  motorL.TurnLeft(power);
}
void backward(int power) {
  BTS7960MotorDriver motorR(7, 8, 9, 10);
  BTS7960MotorDriver motorL(2, 4, 5, 6);

  motorR.Ready();
  motorL.Ready();

  motorR.TurnLeft(power);
  motorL.TurnRight(power);
}

void turnLeft(int power) {
  BTS7960MotorDriver motorR(7, 8, 9, 10);
  BTS7960MotorDriver motorL(2, 4, 5, 6);

  motorR.Ready();
  motorL.Ready();

  motorR.TurnRight(power);
  motorL.TurnRight(power);
}
void turnRight(int power) {
  BTS7960MotorDriver motorR(7, 8, 9, 10);
  BTS7960MotorDriver motorL(2, 4, 5, 6);

  motorR.Ready();
  motorL.Ready();

  motorR.TurnLeft(power);
  motorL.TurnLeft(power);
}
void motorsOFF() {
  BTS7960MotorDriver motorR(7, 8, 9, 10);
  BTS7960MotorDriver motorL(2, 4, 5, 6);

  motorR.Ready();
  motorL.Ready();

  motorR.Stop();
  motorL.Stop();
}

boolean fliped_over() {   // return true if flipped
  // buzzer on
  return false;
}
