//  Copyright (C) Sven Rosvall (sven@rosvall.ie)
//  This file is part of CANNX project on https://github.com/SvenRosvall/CANNX
//  Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//  The full licence can be found at: http://creativecommons.org/licenses/by-nc-sa/4.0

/*
      3rd party libraries needed for compilation: (not for binary-only distributions)

      Streaming   -- C++ stream style output, v5, (http://arduiniana.org/libraries/streaming/)
      VLCB        -- VLCB library for communicating on a CBUS network.
      ACAN2515    -- library to support the MCP2515/25625 CAN controller IC
*/

// 3rd party libraries
#include <Streaming.h>

// VLCB library header files
#include <VLCB.h>
#include <CAN2515.h>               // Chosen CAN controller

// forward function declarations
void eventhandler(byte, const VLCB::VlcbMessage *);
void printConfig();

// constants
const byte VER_MAJ = 1;             // code major version
const char VER_MIN = 'a';           // code minor version
const byte VER_BETA = 0;            // code beta sub-version
const byte MANUFACTURER = MANU_DEV; // for boards in development.
const byte MODULE_ID = 101;          // VLCB module type

const byte LED_GRN = 4;             // VLCB green Unitialised LED pin
const byte LED_YLW = 7;             // VLCB yellow Normal LED pin
const byte SWITCH0 = 8;             // VLCB push button switch pin

// module name, must be at most 7 characters.
char mname[] = "NX";

VLCB::CAN2515 can2515;                  // CAN transport object

// Service objects
VLCB::LEDUserInterface ledUserInterface(LED_GRN, LED_YLW, SWITCH0);
VLCB::SerialUserInterface serialUserInterface;
VLCB::MinimumNodeServiceWithDiagnostics mnService;
VLCB::CanServiceWithDiagnostics canService(&can2515);
VLCB::NodeVariableService nvService;
VLCB::ConsumeOwnEventsService coeService;
VLCB::EventConsumerService ecService;
VLCB::EventTeachingService etService;
VLCB::EventProducerService epService;

//
/// setup VLCB - runs once at power on from setup()
//
void setupVLCB()
{
  VLCB::checkStartupAction(LED_GRN, LED_YLW, SWITCH0);

  VLCB::setServices({
    &mnService, &ledUserInterface, &serialUserInterface, &canService, &nvService,
    &ecService, &epService, &etService, &coeService});

  // set config layout parameters
  VLCB::setNumNodeVariables(1);
  VLCB::setMaxEvents(32);
  VLCB::setNumEventVariables(20);

  // set module parameters
  VLCB::setVersion(VER_MAJ, VER_MIN, VER_BETA);
  VLCB::setModuleId(MANUFACTURER, MODULE_ID);

  // set module name
  VLCB::setName(mname);

  // register our VLCB event handler, to receive event messages of learned events
  ecService.setEventHandler(eventhandler);

  // configure and start CAN bus and VLCB message processing
  can2515.setNumBuffers(2, 2);      // more buffers = more memory used, fewer = less
  can2515.setOscFreq(16000000UL);   // select the crystal frequency of the CAN module
  can2515.setPins(10, 2);           // select pins for CAN bus CE and interrupt connections
  if (!can2515.begin())
  {
    Serial << F("> error starting VLCB") << endl;
  }

  // initialise and load configuration
  VLCB::begin();

  Serial << F("> mode = (") << _HEX(VLCB::getCurrentMode()) << ") " << VLCB::Configuration::modeString(VLCB::getCurrentMode());
  Serial << F(", CANID = ") << VLCB::getCANID();
  Serial << F(", NN = ") << VLCB::getNodeNum() << endl;

  // show code version and copyright notice
  printConfig();
}

//
/// setup - runs once at power on
//
void setup()
{
  Serial.begin (115200);
  Serial << endl << endl << F("> ** CANNX ** ") << __FILE__ << endl;

  setupVLCB();

  // end of setup
  Serial << F("> ready") << endl << endl;
}

//
/// loop - runs forever
//
void loop()
{
  //
  /// do VLCB message, switch and LED processing
  //
  VLCB::process();

  // bottom of loop()
}

//
/// user-defined event processing function
/// called from the VLCB library when a learned event is received
/// it receives the event table index and the CAN frame
//
void eventhandler(byte index, const VLCB::VlcbMessage *msg)
{
  byte evval = VLCB::getEventEVval(index, 2);  //read ev2 because ev1 defines producer.
  // Event Off op-codes have odd numbers.
  bool ison = (msg->data[0] & 0x01) == 0;

  Serial << F("> event handler: index = ") << index << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;
  Serial << F("> EV2 = ") << evval << endl;

  
  // TODO: Check timer. Is this the first button press?
  
  // TODO: First button press: Collect and save possible route set for this button as specified in EVs.
  
  // TODO: Subsequent button press: Collect possible route set for this button.
  // TODO: Match route set with previous route set. (Intersection)
  
  // TODO: How many matches in intersected set?
  // TODO: 0) Ignore button presses.
  // TODO: 1) Send route event.
  // TODO: >=2) Log diagnostic, print warning message. Choose first matching route.
}

//
/// print code version config details and copyright notice
//
void printConfig()
{
  // code version
  Serial << F("> code version = ") << VER_MAJ << VER_MIN << F(" beta ") << VER_BETA << endl;
  Serial << F("> compiled on ") << __DATE__ << F(" at ") << __TIME__ << F(", compiler ver = ") << __cplusplus << endl;

  // copyright
  Serial << F("> © Sven Rosvall (MERG 3777) 2026") << endl;
}
