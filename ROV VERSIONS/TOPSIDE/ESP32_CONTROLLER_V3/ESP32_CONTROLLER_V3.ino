//LIBARIES
#include <Bluepad32.h>
#include<Wire.h>

//CONTROLLER DEFINITIONS
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int lmotor;
int rmotor;
int vert;
int desc;
int pad;
int stop;
int sweep;


//I2C DEFINITIONS
#define slaveAddress 4

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
//FORMULATE MOTOR INPUTS AND STOP
  lmotor = map(ctl->axisY(), -511, 512, 255, 0);
  //Serial.println(lmotor);
  rmotor = map(ctl->axisRY(), -511, 512, 255, 0);
  //Serial.println(rmotor);
  vert = map(ctl->throttle(), 0, 1023, 0, 255);
  //Serial.println(vert);
  desc = map(ctl->brake(), 0, 1023, 0, 255);
  //Serial.println(desc)
  pad = ctl->dpad();
  //Serial.println(pad);
  sweep = (ctl->r1());
  //Serial.println(sweep);
  stop = (ctl->l1());
  //Serial.println(stop);
  
   //STOP
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
  //CONTROLLER SETUP
    Serial.begin(57600); //TRY 115000 BAUD
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
  //I2C SETUP
  Wire.begin();
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    delay(150);

    //I2C
    byte dataArray[7] = {lmotor, rmotor, vert, desc, pad, stop, sweep};
    int size = sizeof(dataArray);
    Wire.beginTransmission(slaveAddress); //address is queued for checking if the slave is present
      for (int i=0; i<size; i++) {
        Wire.write(dataArray[i]);  //data bytes are queued in local buffer
      }
    Wire.endTransmission(); //all the above queued bytes are sent to slave on ACK handshaking
}


  
