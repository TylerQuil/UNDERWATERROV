//LIBARIES
#include <Bluepad32.h>


ControllerPtr myControllers[BP32_MAX_GAMEPADS];

float lmotor;
float rmotor;
float vert;
float desc;
float pad;
bool sweep;
#define LEFTMOTOR 16
#define RIGHTMOTOR 2
#define FRONTMOTOR 15
#define BACKMOTOR 17
#define TOLERANCE 10

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

//void dumpGamepad(ControllerPtr ctl) {
    //Serial.printf(
        //"idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        //"misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        //ctl->index(),        // Controller Index
       //ctl->dpad(),         // D-pad
        //ctl->buttons(),      // bitmask of pressed buttons
        //ctl->axisX(),        // (-511 - 512) left X Axis
        //ctl->axisY(),        // (-511 - 512) left Y axis
        //ctl->axisRX(),       // (-511 - 512) right X axis
        //ctl->axisRY(),       // (-511 - 512) right Y axis
        //ctl->brake(),        // (0 - 1023): brake button
        //ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        //ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    //);
//}

void processGamepad(ControllerPtr ctl) {
//FORMULATE MOTOR INPUTS AND KILL
  lmotor = map(ctl->axisY(), -511, 512, 255, -255);
  Serial.println(lmotor);
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

}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    delay(150);

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

  if (vert >= TOLERANCE ^ vert <= -TOLERANCE) {
    analogWrite(FRONTMOTOR, vert);
    analogWrite(BACKMOTOR, vert);
  }
    else {
      analogWrite(FRONTMOTOR, 0);
      analogWrite(BACKMOTOR, 0);
    }
}
