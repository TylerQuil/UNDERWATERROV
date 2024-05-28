//LIBRARIES
#include <Wire.h>
#include <ESP32Servo.h>
#include <CytronMotorDriver.h>

//I2C TOPSIDE
#define slaveAddress 4  //you have to assign an 8-bit address to Slave
byte dataArray[7];
int lmotorf;
int lmotor;
int rmotorf;
int rmotor;
int vert;
int desc;
int updown;
int pad;
int stop;
int sweep;

//MOTOR FUNCTION
#define LEFTMOTOR1 32
#define LEFTMOTOR2 33
#define RIGHTMOTOR1 25
#define RIGHTMOTOR2 26
#define FRONTMOTOR1 16
#define FRONTMOTOR2 4
#define BACKMOTOR1 0
#define BACKMOTOR2 2
#define TOLERANCE 50
CytronMD motor1(PWM_PWM, LEFTMOTOR1, LEFTMOTOR2);
CytronMD motor2(PWM_PWM, RIGHTMOTOR1, RIGHTMOTOR2);
CytronMD motor3(PWM_PWM, FRONTMOTOR1, FRONTMOTOR2);
CytronMD motor4(PWM_PWM, BACKMOTOR1, BACKMOTOR2);

//SWEEP SERVO
Servo myservo1;
int pos1 = 0;
int servo1Pin = 12;
int sweeptime = 500;
int minimum = 1000;
int maximum = 2000;

//CAMERA SERVOS
Servo myservo2;
Servo myservo3;
int pos2 = 0;
int servo2Pin = 27;
int pos3 = 0;
int servo3Pin = 14;


void setup() {
  //I2C
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);

  //SERVOS
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo1.setPeriodHertz(50);   // standard 50 hz servo
  myservo2.setPeriodHertz(50);
  myservo3.setPeriodHertz(50);
}

void loop() {
  //SWEEP SERVO
  myservo1.attach(servo1Pin, minimum, maximum);
  if (sweep == 1) {
    pos1 = 180; // goes from 0 degrees to 180 degrees in steps of 1 degree
    myservo1.write(pos1);    // tell servo to go to position in variable 'pos1'
    delay(sweeptime);
  }
    else {
      pos1 = 0;
      myservo1.write(pos1);
    } 
  //Serial.println(pos1);

 //CAMERA SERVOS
  if (pad == 4 || pad == 5 || pad == 6) {
    pos2 = pos2 + 1;
    myservo2.write(pos2);
  }
  if (pad == 8 || pad == 9 || pad == 10) {
    pos2 = pos2 - 1;
    myservo2.write(pos2);
  }
  if (pad == 9 || pad == 1 || pad == 5) {
    pos3 = pos3 + 1;
    myservo2.write(pos3);
  }
   if (pad == 10 || pad == 2 || pad == 6) {
    pos3 = pos3 - 1;
    myservo2.write(pos3);
  }


  //MOTOR CONTROL WITH DEADZONE
  if (lmotor >= TOLERANCE ^ lmotor <= -TOLERANCE) {
   motor1.setSpeed(lmotor);
  }
   else {
    motor1.setSpeed(0);
    }

  if (rmotor >= TOLERANCE ^ rmotor <= -TOLERANCE) {
    motor2.setSpeed(rmotor);
  }
    else {
      motor2.setSpeed(0);
    }

if (abs(desc) >= vert) {updown = desc;}
  else {updown = vert;}
  if (updown >= TOLERANCE ^ updown <= -TOLERANCE) {
    motor3.setSpeed(updown);
    motor4.setSpeed(updown);
  }
    else {
      motor3.setSpeed(0);
      motor4.setSpeed(0);
    }
}

//I2C DATA PULL FROM TOPSIDE
void receiveEvent(int howMany) {
    for (int i=0; i<howMany; i++) {
    dataArray[i] = Wire.read();
    lmotorf = dataArray[0];
    lmotor = map(lmotorf, 0, 255, -255, 255);
    Serial.print("lmotor = ");
    Serial.println(lmotor);
    rmotorf = dataArray[1];
    rmotor = map(rmotorf, 0, 255, -255, 255);
    //Serial.print("rmotor = ");
    //Serial.println(rmotor);
    vert = dataArray[2];
    //Serial.print("vert = ");
    //Serial.println(vert);
    desc = - dataArray[3];
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

