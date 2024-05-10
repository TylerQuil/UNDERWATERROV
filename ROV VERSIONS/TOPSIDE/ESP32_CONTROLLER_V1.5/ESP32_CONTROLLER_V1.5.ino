//LIBARIES
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <CytronMotorDriver.h>
#include <Wire.h>


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int lmotor;
int rmotor;
int vert;
int desc;
int updown;
int pad;
int stop;
int sweep;

//MOTOR FUNCTION
#define LEFTMOTOR1 25
#define LEFTMOTOR2 26
#define RIGHTMOTOR1 32
#define RIGHTMOTOR2 33 //23
#define FRONTMOTOR1 16
#define FRONTMOTOR2 4
#define BACKMOTOR1 0
#define BACKMOTOR2 2
#define TOLERANCE 50
CytronMD motorh1(PWM_PWM, LEFTMOTOR1, LEFTMOTOR2);
CytronMD motorh2(PWM_PWM, RIGHTMOTOR1, RIGHTMOTOR2);
CytronMD motorv1(PWM_PWM, FRONTMOTOR1, FRONTMOTOR2);
CytronMD motorv2(PWM_PWM, BACKMOTOR1, BACKMOTOR2);

//SWEEP SERVO
Servo myservo1;
int pos1 = 0;
int servo1Pin = 13;
int sweeptime = 500;
int minimum = 1000;
int maximum = 2000;

//CAMERA SERVOS
Servo myservo2;
Servo myservo3;
int pos2 = 0;
int servo2Pin = 12;
int pos3 = 0;
int servo3Pin = 11;


// CALLED ON NEW GAMEPAD CONNECTION (UP TO 4 CONTROLLER)
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void processGamepad(ControllerPtr ctl) {
//FORMULATE MOTOR INPUTS AND KILL
  lmotor = map(ctl->axisY(), -511, 512, 255, -255);
  //Serial.println(lmotor);
  rmotor = map(ctl->axisRY(), -511, 512, 255, -255);
  //Serial.println(rmotor);
  vert = map(ctl->throttle(), 0, 1023, 0, 255);
  //Serial.println(vert);
  desc = map(ctl->brake(), 0, 1023, 0, -255);
  //Serial.println(desc)
  Serial.print(ctl->battery());
  pad = ctl->dpad();
  //Serial.println(pad);

   //KILL
    if (ctl->l1()) {
      Serial.println("ROV OPERATIONS PAUSED");
      ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    delay(500);
    exit(0);
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

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

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    delay(150);

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
   motorh1.setSpeed(lmotor);
   Serial.println(lmotor);
  }
   else {
    motorh1.setSpeed(0);
    }

  if (rmotor >= TOLERANCE ^ rmotor <= -TOLERANCE) {
    motorh2.setSpeed(rmotor);
    Serial.println(rmotor);
  }
    else {
      motorh2.setSpeed(0);
    }

if (abs(desc) >= vert) {updown = desc;}
  else {updown = vert;}
  if (updown >= TOLERANCE ^ updown <= -TOLERANCE) {
    motorv1.setSpeed(updown);
    motorv2.setSpeed(updown);
  }
    else {
      motorv1.setSpeed(0);
      motorv2.setSpeed(0);
    }
}
