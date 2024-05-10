//LIBRARIES
#include<Wire.h>

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
#define LEFTMOTOR 16
#define RIGHTMOTOR 17
#define FRONTMOTOR 4
#define BACKMOTOR 5
#define TOLERANCE 50


void setup() {
  //I2C
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
}

void loop() {
  //MOTOR CONTROL WITH DEADZONE
  if (lmotor >= TOLERANCE ^ lmotor <= -TOLERANCE) {
   analogWrite(LEFTMOTOR, lmotor);
  }
   else {
    analogWrite(LEFTMOTOR, 0);
    }

  if (rmotor >= TOLERANCE ^ rmotor <= -TOLERANCE) {
    analogWrite(RIGHTMOTOR, rmotor);
  }
    else {
      analogWrite(RIGHTMOTOR, 0);
    }

if (abs(desc) >= vert) {updown = desc;}
  else {updown = vert;}
  if (updown >= TOLERANCE ^ updown <= -TOLERANCE) {
    analogWrite(FRONTMOTOR, vert);
    analogWrite(BACKMOTOR, vert);
  }
    else {
      analogWrite(FRONTMOTOR, 0);
      analogWrite(BACKMOTOR, 0);
    }
}

//I2C DATA PULL FROM TOPSIDE
void receiveEvent(int howMany) {
    for (int i=0; i<howMany; i++) {
    dataArray[i] = Wire.read();
    lmotorf = dataArray[0];
    lmotor = map(lmotorf, 0, 255, -255, 255);
    //Serial.print("lmotor = ");
    //Serial.println(lmotor);
    rmotorf = dataArray[1];
    rmotor = map(rmotorf, 255, 0, 255, -255);
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

