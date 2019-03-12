//Mixed Values

#include <btsmotordriver.h>

  int RC1;
  int RC2;
  int mode;
  
void setup() {
  pinMode(12, INPUT);
  pinMode(13, INPUT);

  Serial.begin(57600);
}
void loop() {
  float timing = 25000;
  digitalWrite(14, HIGH);
  pulseIn(12, HIGH, timing);
  pulseIn(13, HIGH, timing);

  RC1 = pulseIn(12, HIGH, timing);
  RC2 = pulseIn(13, HIGH, timing);

  Serial.print("RC1: ");
  Serial.println(RC1);
  Serial.print("RC2: ");
  Serial.println(RC2);
/*
  int input1 = RC1;
  int power1 = input1 / 4;
  
  int input2 = RC2;
  int power2 = input2 / 4;
  */
  
  int power = 150;
  int power1 = power;
  int power2 = power;
  if(RC1 > 1600){
    Serial.println("ENTERED TURN RIGHT");
    turnRight(power1);
    delay(200);
    RC1 = pulseIn(12, HIGH, timing);
    RC2 = pulseIn(13, HIGH, timing);

  }
  if(RC1 < 1400){
    Serial.println("ENTERED TURN LEFT");
    turnLeft(power1);
    delay(200);
    RC1 = pulseIn(12, HIGH, timing);
    RC2 = pulseIn(13, HIGH, timing);
  }
 
  if(RC2 < 1400){
    Serial.println("ENTERED BACKWARD");
    backward(power2);
    delay(200);
    RC1 = pulseIn(12, HIGH, timing);
    RC2 = pulseIn(13, HIGH, timing);
  }
   if(RC2 > 1600){
    Serial.println("ENTERED FORWARD");
    forward(power2);
    delay(200);
    RC1 = pulseIn(12, HIGH, timing);
    RC2 = pulseIn(13, HIGH, timing);
  }
  else{
   // motorsOFF();
  }
 
}
void activateGPSMode(){
  motorsOFF();
}
void  activateRCMode(){

}
void forward(int power){
  BTS7960MotorDriver motorR(7,8,9,10);
  BTS7960MotorDriver motorL(2,4,5,6);

  motorR.Ready();
  motorL.Ready();
  
  motorR.TurnRight(power);
  motorL.TurnLeft(power);
}
void backward(int power){
  BTS7960MotorDriver motorR(7,8,9,10);
  BTS7960MotorDriver motorL(2,4,5,6);

  motorR.Ready();
  motorL.Ready();
  
  motorR.TurnLeft(power);
  motorL.TurnRight(power);
}

void turnLeft(int power){
  BTS7960MotorDriver motorR(7,8,9,10);
  BTS7960MotorDriver motorL(2,4,5,6);

  motorR.Ready();
  motorL.Ready();
  
  motorR.TurnRight(power);
  motorL.TurnRight(power);
}
void turnRight(int power){
  BTS7960MotorDriver motorR(7,8,9,10);
  BTS7960MotorDriver motorL(2,4,5,6);

  motorR.Ready();
  motorL.Ready();
  
  motorR.TurnLeft(power);
  motorL.TurnLeft(power); 
}
void motorsOFF(){
  BTS7960MotorDriver motorR(7,8,9,10);
  BTS7960MotorDriver motorL(2,4,5,6);

  motorR.Ready();
  motorL.Ready();
  
  motorR.Stop();
  motorL.Stop();
}

boolean fliped_over(){    // return true if flipped
  // buzzer on 
  return false;
}
