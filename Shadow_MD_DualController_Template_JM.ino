
// =======================================================================================
//        SHADOW_PS4:  Small Handheld Arduino Droid Operating Wand
// =======================================================================================
//                          Last Revised Date: 7/19/19
//                             Revised By: iceowl

// lots and lots of more documentation needed
// I wrote almost none of it.  It was all written by the club and others
//  I just did mods.
//  you *must* use the USB_HOST_SHIELD libs I uploaded for this to work


//                Inspired by the PADAWAN / KnightShade SHADOW effort
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it for
//         your personal use and the personal use of other astromech club members.
//
//         This program is distributed in the hope that it will be useful
//         as a courtesy to fellow astromech club members wanting to develop
//         their own droid control system.
//
//         IT IS OFFERED WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         You are using this software at your own risk and, as a fellow club member, it is
//         expected you will have the proper experience / background to handle and manage that
//         risk appropriately.  It is completely up to you to insure the safe operation of
//         your droid and to validate and test all aspects of your droid control system.
//
// =======================================================================================
//
//
//   PS4 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//   For more information visit my blog: http://blog.tkjelectronics.dk/  at
//
//
//    https://github.com/felis/USB_Host_Shield
//
//
//
// =======================================================================================
#define SHADOW_DEBUG   1    //uncomment this for console DEBUG output
#define SHADOW_VERBOSE 1   //uncomment this for console VERBOSE output


#define SYREN_ADDR         129      // Serial Address for Dome Syren
#define SABERTOOTH_ADDR    128      // Serial Address for Foot Sabertooth
#define KANGAROO_ADDR      130      // Serial Address for Kangaroo 2-3-2

#define PAIR_PIN           40
#define PAIR_PIN_HIGH      38

#include <PS4BT.h>
#include <PS4USB.h>
#include <SPI.h>
#include <Sabertooth.h>
#include <Kangaroo.h>
#include <usbhub.h>


USB    Usb;
BTD    Btd(&Usb);
PS4BT  *myPS4;



byte drivespeed1 = 30;   //For Speed Setting (Normal): set this to whatever speeds works for you. 0-stop, 127-full speed.
byte drivespeed2 = 110;  //For Speed Setting (Over Throttle): set this for when needing extra power. 0-stop, 127-full speed.
byte domespeed   = 50;
byte turnspeed   = 30;      // the higher this number the faster it will spin in place, lower - the easier to control.
// Recommend beginner: 40 to 50, experienced: 50+, I like 75

//byte domespeed = 100;    // If using a speed controller for the dome, sets the top speed
// Use a number up to 127

byte ramping = 1;        // Ramping- the lower this number the longer R2 will take to speedup or slow down,
// change this by increments of 1

byte joystickFootDeadZoneRange = 15;  // For controllers that centering problems, use the lowest number with no drift
byte joystickDomeDeadZoneRange = 10;  // For controllers that centering problems, use the lowest number with no drift

byte driveDeadBandRange = 10;     // Used to set the Sabertooth DeadZone for foot motors

int invertTurnDirection = -1;   //This may need to be set to 1 for some configurations
int motorControllerBaudRate = 9600; // Set the baud rate for the Syren motor controller

// for packetized options are: 2400, 9600, 19200 and 38400
long previousDomeMillis        = millis();
long previousFootMillis        = millis();
long previousSpeedToggleMillis = millis();
long currentMillis             = millis();

int serialLatency              = 50;   //This is a delay factor in ms to prevent queueing of the Serial data.
//25ms seems approprate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation
int speedToggleButtonCounter   = 0;
int domeToggleButtonCounter    = 0;



uint32_t msgLagTime            = 0;
uint32_t lastMsgTime           = 0;
uint32_t currentTime           = 0;
uint32_t lastLoopTime          = 0;
int      badPS4Data            = 0;

boolean firstMessage = true;
boolean firstLoop    = true;

String output = (char*) malloc(1000); //this seems to work better than the way they have been doing it - it was dropping messages so I did it the old "C" way
boolean isFootMotorStopped        = true;
boolean isDomeMotorStopped        = true;
boolean overSpeedSelected         = false;
boolean isPS4NavigatonInitialized = false;
boolean isStickEnabled            = true;
boolean WaitingforReconnect       = false;
boolean mainControllerConnected   = false;
boolean legsParallel              = false;

// Dome Automation Variables
boolean       domeAutomation      = false;
int           domeTurnDirection   = 1;  // 1 = positive turn, -1 negative turn
float         domeTargetPosition  = 0; // (0 - 359) - degrees in a circle, 0 = home
unsigned long domeStopTurnTime    = 0;    // millis() when next turn should stop
unsigned long domeStartTurnTime   = 0;  // millis() when next turn should start
int           domeStatus          = 0;  // 0 = stopped, 1 = prepare to turn, 2 = turning


byte          action            = 0;
unsigned long DriveMillis       = 0;
int           footDriveSpeed    = 0;
long          minimumK          = 0;
long          maximumK          = 0;

String        btAddress            = "                 ";
String        PS4ControllerAddress = "1C:66:6D:2B:A7:4D";


KangarooSerial    K(Serial2);
KangarooChannel   K1(K, '1', KANGAROO_ADDR);
Sabertooth       *SyR = new Sabertooth(SYREN_ADDR, Serial2);
Sabertooth       *ST  = new Sabertooth(SABERTOOTH_ADDR, Serial2);


void setup()
{


  //Debug Serial for use with USB Debugging

  SPI.begin();
  Serial.begin(115200);
  Serial2.begin(motorControllerBaudRate);

  
  pinMode(PAIR_PIN, INPUT);
  pinMode(PAIR_PIN_HIGH, OUTPUT);
  digitalWrite(PAIR_PIN_HIGH,HIGH);

  
#ifdef SHADOW_DEBUG
  Serial.print(F("\r\nProgram Start"));
#endif
  if (Usb.Init() == -1)
  {
#ifdef SHADOW_DEBUG
    Serial.print(F("\r\nOSC did not start"));
#endif
    while (1); //halt
  }

#ifdef SHADOW_DEBUG
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
#endif


  if (digitalRead(PAIR_PIN) == HIGH) {
    myPS4 = new PS4BT(&Btd, PAIR);
#ifdef SHADOW_DEBUG
    Serial.print(F("\r\nPS4 started in PAIR mode"));
#endif
  } else {
    myPS4  = new PS4BT(&Btd);
#ifdef SHADOW_DEBUG
    Serial.print(F("\r\nPS4 started in NON-PAIR mode"));
#endif
  }

  while (!Btd.isReady()) {
    delay(500);
    Usb.Task();
#ifdef SHADOW_DEBUG
    Serial.print(F("\r\ndongle not ready"));
#endif
  }

  myPS4->attachOnInit(onInitmyPS4); // onInitps4NavFoot is called upon a new connection

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif


  output = "";




#ifdef SHADOW_DEBUG
  Serial.print(F("\r\nDelay 2 secs"));
#endif
  delay(2000);
  K1.start();
  K1.serialTimeout(7000);
  minimumK = K1.getMin().value();
  maximumK = K1.getMax().value();
#ifdef SHADOW_DEBUG
  Serial.print(F("\r\nStart K1 and set serial timeout"));
#endif
  for (int i = 1; i < 2; i++) Usb.Task();
  delay(500);
  K1.p(minimumK + 100).wait();
  K1.powerDown();
#ifdef SHADOW_DEBUG
  Serial.print(F("\r\nLegs Extended"));
#endif


  ST->autobaud();          // Send the autobaud command to the Sabertooth controller(s).
  ST->setTimeout(10);      //DMB:  How low can we go for safety reasons?  multiples of 100ms
  ST->setDeadband(driveDeadBandRange);
  ST->stop();

#ifdef SHADOW_DEBUG
  Serial.print(F("\r\nSabertooth started"));
#endif
  randomSeed(analogRead(0));  // random number seed for dome automation


}

// =======================================================================================
//           Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================



void loop() {

  // this decision allows us to select the PAIR function by tying digital pin 30 to GND.  If it's high (e.g. connected to PAIR_PIN_HIGH) it doesn't pair
  // but simply connects


#ifdef SHADOW_DEBUG
  if (firstLoop) {
    Serial.print(F("\r\nLoop Started"));
    firstLoop = false;
  }
#endif



  if ( !readUSB() ) {
#ifdef SHADOW_DEBUG
    output = "readUSB came back false\n";
    // We have a fault condition that we want to ensure that we do NOT process any controller data!!!
    printOutput();
#endif
    return;
  }

  footMotorDrive();
  domeDrive();
  toggleSettings();

#ifdef SHADOW_DEBUG
  printOutput();
#endif

}

// =======================================================================================
//           footDrive Motor Control Section
// =======================================================================================


boolean ps4FootMotorDrive() {

  int stickSpeed       = 0;
  int turnnum          = 0;
  int joystickPosition = 0;

  if (isPS4NavigatonInitialized) {
    // Additional fault control.  Do NOT send additional commands to Sabertooth if no controllers have initialized.
    if (!isStickEnabled) {

#ifdef SHADOW_VERBOSE
      if ( abs(myPS4->getAnalogHat(LeftHatY) - 128) > joystickFootDeadZoneRange) {
        output += "Drive Stick is disabled\r\n";
      }
#endif
      if (!isFootMotorStopped) {
        ST->stop();
        isFootMotorStopped = true;
        footDriveSpeed     = 0;
#ifdef SHADOW_VERBOSE
        output += "\r\n***Foot Motor STOPPED***\r\n";
#endif
      }
      return false;
    }
    if (myPS4->getButtonPress(L2) || myPS4->getButtonPress(L1)) {

      if (!isFootMotorStopped) {
        ST->stop();
        isFootMotorStopped = true;
        footDriveSpeed     = 0;

      }
#ifdef SHADOW_VERBOSE
      output += "\r\n***Foot Motor STOPPED by L1/L2 ***\r\n";
#endif
      return false;
    }

    joystickPosition = myPS4->getAnalogHat(LeftHatY);

    if (overSpeedSelected) { //Over throttle is selected
      stickSpeed = (map(joystickPosition, 0, 255, -drivespeed2, drivespeed2));
    } else stickSpeed = (map(joystickPosition, 0, 255, -drivespeed1, drivespeed1));


    if ( abs(joystickPosition - 128) < joystickFootDeadZoneRange) {
      // This is RAMP DOWN code when stick is now at ZERO but prior FootSpeed > 20

      if (abs(footDriveSpeed) > 50) {
        if (footDriveSpeed > 0) {
          footDriveSpeed -= 3;

        } else footDriveSpeed += 3;


#ifdef SHADOW_VERBOSE
        output += "ZERO FAST RAMP: footSpeed: ";
        output += footDriveSpeed;
        output += "\nStick Speed: ";
        output += stickSpeed;
        output += "\n\r";
#endif

      } else if (abs(footDriveSpeed) > 20) {
        if (footDriveSpeed > 0) {
          footDriveSpeed -= 2;
        } else footDriveSpeed += 2;


#ifdef SHADOW_VERBOSE
        output += "ZERO MID RAMP: footSpeed: ";
        output += footDriveSpeed;
        output += "\nStick Speed: ";
        output += stickSpeed;
        output += "\n\r";
#endif
      } else footDriveSpeed = 0;

    } else {
      isFootMotorStopped = false;
      if (footDriveSpeed < stickSpeed) {
        if ((stickSpeed - footDriveSpeed) > (ramping + 1)) {
          footDriveSpeed += ramping;

#ifdef SHADOW_VERBOSE
          output += "RAMPING UP: footSpeed: ";
          output += footDriveSpeed;
          output += "\nStick Speed: ";
          output += stickSpeed;
          output += "\n\r";
#endif
        } else  footDriveSpeed = stickSpeed;
      } else if (footDriveSpeed > stickSpeed) {
        if ((footDriveSpeed - stickSpeed) > (ramping + 1)) {
          footDriveSpeed -= ramping;
#ifdef SHADOW_VERBOSE
          output += "RAMPING DOWN: footSpeed: ";
          output += footDriveSpeed;
          output += "\nStick Speed: ";
          output += stickSpeed;
          output += "\n\r";
#endif
        } else footDriveSpeed = stickSpeed;
      } else {
        footDriveSpeed = stickSpeed;
      }
    }

    turnnum = (myPS4->getAnalogHat(LeftHatX));

    //TODO:  Is there a better algorithm here?
    if ( abs(footDriveSpeed) > 50)
      turnnum = (map(myPS4->getAnalogHat(LeftHatX), 54, 200, -(turnspeed / 4), (turnspeed / 4)));
    else if (turnnum <= 200 && turnnum >= 54)
      turnnum = (map(myPS4->getAnalogHat(LeftHatX), 54, 200, -(turnspeed / 3), (turnspeed / 3)));
    else if (turnnum > 200)
      turnnum = (map(myPS4->getAnalogHat(LeftHatX), 201, 255, turnspeed / 3, turnspeed));
    else if (turnnum < 54)
      turnnum = (map(myPS4->getAnalogHat(LeftHatX), 0, 53, -turnspeed, -(turnspeed / 3)));

    if (abs(turnnum) > 5) {
      isFootMotorStopped = false;
    }
    currentMillis = millis();

    if ( (currentMillis - previousFootMillis) > serialLatency  ) {
      if (footDriveSpeed != 0 || abs(turnnum) > 5) {

#ifdef SHADOW_VERBOSE
        output += "Motor: FootSpeed: ";
        output += footDriveSpeed;
        output += "\nTurnnum: ";
        output += turnnum;
        output += "\nTime of command: ";
        output += millis();
#endif

        ST->turn(turnnum * invertTurnDirection);
        ST->drive(footDriveSpeed);

      } else {
        if (!isFootMotorStopped) {
          ST->stop();
          isFootMotorStopped = true;
          footDriveSpeed = 0;

#ifdef SHADOW_VERBOSE
          output += "\r\n***Foot Motor STOPPED***\r\n";
#endif
        }
      }
      // The Sabertooth won't act on mixed mode packet serial commands until
      // it has received power levels for BOTH throttle and turning, since it
      // mixes the two together to get diff-drive power levels for both motors.

      previousFootMillis = currentMillis;
      return true; //we sent a foot command
    }
  }
  return false;
}

void footMotorDrive() {


  //Flood control prevention
  if ((millis() - previousFootMillis) < serialLatency) return;

  if (myPS4->connected()) ps4FootMotorDrive();

}




// =======================================================================================
//                               Toggle Control Section
// =======================================================================================

void ps4ToggleSettings()
{

  // enable / disable drive stick
  if (myPS4->getButtonPress(PS) && myPS4->getButtonClick(CROSS))
  {

#ifdef SHADOW_DEBUG
    output += "Disabling the DriveStick\r\n";
    output += "Stopping Motors";
#endif

    ST->stop();
    isFootMotorStopped = true;
    isStickEnabled = false;
    footDriveSpeed = 0;
  }

  if (myPS4->getButtonPress(PS) && myPS4->getButtonClick(CIRCLE))
  {
#ifdef SHADOW_DEBUG
    output += "Enabling the DriveStick\r\n";
#endif
    isStickEnabled = true;
  }

  // Enable and Disable Overspeed
  if (myPS4->getButtonPress(L3) && myPS4->getButtonPress(L1) && isStickEnabled) {

    if ((millis() - previousSpeedToggleMillis) > 1000) {
      speedToggleButtonCounter = 0;
      previousSpeedToggleMillis = millis();
    }
    speedToggleButtonCounter += 1;
    if (speedToggleButtonCounter == 1) {
      if (!overSpeedSelected) {
        overSpeedSelected = true;

#ifdef SHADOW_VERBOSE
        output += "Over Speed is now: ON";
#endif

      } else {
        overSpeedSelected = false;

#ifdef SHADOW_VERBOSE
        output += "Over Speed is now: OFF";
#endif
      }
    }
  }

  if (myPS4->getButtonPress(CROSS) && myPS4->getButtonClick(SQUARE) && !legsParallel) {
#ifdef SHADOW_DEBUG
    output += "Parallel Leg Mode\r\n";
#endif


    K1.p(maximumK + 50).wait();
    delay(500);
    K1.powerDown();
    legsParallel = true;
  }

  if (myPS4->getButtonPress(CROSS) && myPS4->getButtonClick(CIRCLE) && legsParallel) {
#ifdef SHADOW_DEBUG
    output += "Tripod Leg Mode\r\n";
#endif

    K1.p(minimumK + 100).wait();
    delay(500);
    K1.powerDown();
    legsParallel = false;
  }
}

void toggleSettings() {
  if (myPS4->connected()) ps4ToggleSettings();
}


void onInitmyPS4() {

  myPS4->setLed(Blue);
  btAddress = getLastConnectedBtMAC();
  if (btAddress == PS4ControllerAddress) {
    myPS4->setLed(Blue);
    isPS4NavigatonInitialized = true;
    badPS4Data = 0;
    mainControllerConnected = true;
    WaitingforReconnect = true;
#ifdef SHADOW_DEBUG
    output += "\r\nWe have our controller connected with address ";
    output += btAddress;
    output += " \n";
#endif

  } else {

    // Prevent connection from anything but the MAIN controllers
#ifdef SHADOW_DEBUG
    output += "\r\nWe have an invalid controller trying to connect \r\n";
#endif

    ST->stop();
    isFootMotorStopped = true;
    footDriveSpeed     = 0;
    myPS4->setLed(Red);
    delay(2000);
    myPS4->setLedOff();
    myPS4->disconnect();
    isPS4NavigatonInitialized = false;
    mainControllerConnected   = false;


  }
}

boolean criticalFaultDetect() {
  if (myPS4->connected()) {

    currentTime = millis();
    lastMsgTime = myPS4->getLastMessageTime();
    msgLagTime = currentTime - lastMsgTime;

    if (WaitingforReconnect) {
      if (msgLagTime < 200) {
        WaitingforReconnect = false;
      }
      lastMsgTime = currentTime;
    }
    if ( currentTime >= lastMsgTime) {
      msgLagTime = currentTime - lastMsgTime;
    } else {
      msgLagTime = 0;
    }

    if (msgLagTime > 300 && !isFootMotorStopped) {
#ifdef SHADOW_DEBUG
      output += "It has been 300ms since we heard from the PS4 Controller\r\n";
      output += "Shut downing motors, and watching for a new PS4 message\r\n";
#endif
      ST->stop();
      isFootMotorStopped = true;
      footDriveSpeed = 0;
    }

    if ( msgLagTime > 10000 )  {
#ifdef SHADOW_DEBUG
      output += "It has been 10s since we heard from the PS4 Controller\r\n";
      output += "msgLagTime:";
      output += msgLagTime;
      output += "  lastMsgTime:";
      output += lastMsgTime;
      output += "  millis:";
      output += millis();
      output += "\r\nDisconnecting the Foot controller.\r\n";
#endif
      ST->stop();
      isFootMotorStopped = true;
      footDriveSpeed = 0;
      myPS4->disconnect();
      WaitingforReconnect = true;
      return true;
    }

  }
  return false;
}



String getLastConnectedBtMAC() {
  String btAddress = "";
  for (int8_t i = 5; i > 0; i--) {
    if (Btd.disc_bdaddr[i] < 0x10) {
      btAddress += "0";
    }
    btAddress += String(Btd.disc_bdaddr[i], HEX);
    btAddress += (":");
  }
  btAddress += String(Btd.disc_bdaddr[0], HEX);
  btAddress.toUpperCase();
  return btAddress;
}

// =======================================================================================
//           domeDrive Motor Control Section
// =======================================================================================

int ps4DomeDrive() {

  int domeRotationSpeed = 0;
  int joystickPosition = myPS4->getAnalogHat(RightHatX);

  domeRotationSpeed = (map(joystickPosition, 0, 255, -domespeed, domespeed));

  if ( abs(joystickPosition - 128) < joystickDomeDeadZoneRange )
    domeRotationSpeed = 0;

  if (domeRotationSpeed != 0 && domeAutomation == true)   {// Turn off dome automation if manually moved

    domeAutomation = false;
    domeStatus = 0;
    domeTargetPosition = 0;

#ifdef SHADOW_VERBOSE
    output += "Dome Automation OFF\r\n";
#endif

  }

  return domeRotationSpeed;
}

void rotateDome(int domeRotationSpeed, String mesg) {

  //Constantly sending commands to the SyRen (Dome) is causing foot motor delay.
  //Lets reduce that chatter by trying 3 things:
  // 1.) Eliminate a constant stream of "don't spin" messages (isDomeMotorStopped flag)
  // 2.) Add a delay between commands sent to the SyRen (previousDomeMillis timer)
  // 3.) Switch to real UART on the MEGA (Likely the *CORE* issue and solution)
  // 4.) Reduce the timout of the SyRen - just better for safety!

  currentMillis = millis();
  if ( (!isDomeMotorStopped || domeRotationSpeed != 0) && ((currentMillis - previousDomeMillis) > (2 * serialLatency) ) ) {
    if (domeRotationSpeed != 0) {
      isDomeMotorStopped = false;

#ifdef SHADOW_VERBOSE
      output += "Dome rotation speed: ";
      output += domeRotationSpeed;
#endif

      SyR->motor(domeRotationSpeed);

    } else {
      isDomeMotorStopped = true;

#ifdef SHADOW_VERBOSE
      output += "\n\r***Dome motor is STOPPED***\n\r";
#endif

      SyR->stop();
    }

    previousDomeMillis = currentMillis;
  }
}

void domeDrive() {

  //Flood control prevention
  //This is intentionally set to double the rate of the Dome Motor Latency
  if ((millis() - previousDomeMillis) < (2 * serialLatency) ) return;

  int domeRotationSpeed  = 0;
  int ps4NavControlSpeed = 0;

  if (myPS4->connected()) {

    ps4NavControlSpeed = ps4DomeDrive();
    domeRotationSpeed  = ps4NavControlSpeed;
    rotateDome(domeRotationSpeed, "Controller Move");

  } else if (myPS4->getButtonPress(L2)) {

    ps4NavControlSpeed = ps4DomeDrive();
    domeRotationSpeed  = ps4NavControlSpeed;
    rotateDome(domeRotationSpeed, "Controller Move");

  } else  {
    if (!isDomeMotorStopped)  {
      SyR->stop();
      isDomeMotorStopped = true;
    }
  }
}


boolean readUSB() {

  Usb.Task();

  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  if (myPS4->connected()) {
    if (criticalFaultDetect()) {
      output += " ***FAULT*** ";
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      printOutput();
      return false;
    }
  } else if (!isFootMotorStopped) {
#ifdef SHADOW_DEBUG
    output += "No foot controller was found\r\n";
    output += "Shuting down motors, and watching for a new ps4 foot message\r\n";
#endif
    ST->stop();
    isFootMotorStopped = true;
    footDriveSpeed = 0;
    WaitingforReconnect = true;
  }
  return true;
}

void printOutput() {
  if (output != "") {
#ifdef SHADOW_DEBUG
    Serial.println(output);
#endif
    output = ""; // Reset output string
  }
}
