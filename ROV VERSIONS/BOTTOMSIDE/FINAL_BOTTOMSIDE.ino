//LIBRARIES
#include <Wire.h>
#include <CytronMotorDriver.h>
#include <ESP32Servo.h>

//I2C TOPSIDE
#define slaveAddress 4  //you have to assign an 8-bit address to Slave
byte dataArray[7];
int front ,frontmap, up, upmap;
int left,right,turn, pad, stop;
int sweep;

//MOTOR FUNCTION
#define LEFTMOTOR1 0
#define LEFTMOTOR2 4
#define RIGHTMOTOR1 16
#define RIGHTMOTOR2 2
#define FRONTMOTOR1 32
#define FRONTMOTOR2 33
#define BACKMOTOR1 25
#define BACKMOTOR2 26
#define TOLERANCE 30
CytronMD motor1(PWM_PWM, LEFTMOTOR1, LEFTMOTOR2);
CytronMD motor2(PWM_PWM, RIGHTMOTOR1, RIGHTMOTOR2);
CytronMD motor3(PWM_PWM, FRONTMOTOR1, FRONTMOTOR2);
CytronMD motor4(PWM_PWM, BACKMOTOR1, BACKMOTOR2);

//SWEEP SERVO
Servo servo1;
int servo1Pin = 27;
int pos1 = 0;
int sweeptime = 200;

//CAMERA SERVOS
Servo servo2;
Servo servo3;
int servo2Pin = 14;
int servo3Pin = 12;
int pos2 = 0;
int pos3 = 0;


void setup() {
  servo1.attach(servo1Pin);
  servo1.write(pos1);
  //gimbal
  servo2.attach(servo2Pin);
  servo2.write(pos2);
  servo3.attach(servo3Pin);
  servo3.write(pos3);
  //I2C
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Serial.begin(57600);
}

void loop() {
  //SWEEP SERVO
  if (sweep == 1) {
    pos1 = 120; // goes from 0 degrees to 180 degrees in steps of 1 degree
    servo1.write(pos1); 
    delay(sweeptime);  
  }
    else {
      pos1 = 0;
      servo1.write(pos1);
    } 
  //Serial.println(pos1);

 //CAMERA SERVOS
  if (pad == 4 || pad == 5 || pad == 6) {
    pos2 = pos2 + 10;
    servo2.write(pos2);
    delay(100)
  }
  if (pad == 8 || pad == 9 || pad == 10) {
    pos2 = pos2 - 10;
    servo2.write(pos2);
    delay(100)
  }
  if (pad == 9 || pad == 1 || pad == 5) {
    pos3 = pos3 + 10;
    servo3.write(pos3);
    delay(100)
  }
   if (pad == 10 || pad == 2 || pad == 6) {
    pos3 = pos3 - 10;
    servo3.write(pos3);
    delay(100)
  }

  if (frontmap >= TOLERANCE || frontmap <= -TOLERANCE) {
   motor1.setSpeed(frontmap);
   motor2.setSpeed(frontmap);
  }
   else {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    }

  if (upmap >= TOLERANCE || upmap <= -TOLERANCE) {
    motor3.setSpeed(upmap);
    motor4.setSpeed(upmap);
  }
  else {
    motor3.setSpeed(0);
    motor4.setSpeed(0);
  }

if (abs(left) >= right) {
    turn = left;
  }
  else {
    turn = right;
  }
  if (turn >= TOLERANCE || turn <= -TOLERANCE) {
    motor1.setSpeed(turn);
    motor2.setSpeed(-turn);
  }
  else {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  }
}

//I2C DATA PULL FROM TOPSIDE
void receiveEvent(int howMany) {
    for (int i=0; i<howMany; i++) {
    dataArray[i] = Wire.read();
    front = dataArray[0];
    frontmap = map(front, 0, 255, -255, 255);
    Serial.print("front = ");
    Serial.println(front);
    up = dataArray[1];
    upmap = map(up, 0, 255, -255, 255);
    //Serial.print("rmotor = ");
    //Serial.println(rmotor);
    left = dataArray[2];
    //Serial.print("vert = ");
    //Serial.println(vert);
    right = - dataArray[3];
    //Serial.print("desc = ");
    //Serial.println(desc);
    pad = dataArray[4];
    //Serial.print("pad = ");
    //Serial.println(pad);
    stop = dataArray[5];
    //Serial.print("stop = ");
    //Serial.println(stop);
    sweep = dataArray[6];
    //Serial.print("sweep = ");
    //Serial.println(sweep);
  }
}

