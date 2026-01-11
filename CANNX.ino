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

const int NumEVs = 20;

VLCB::CAN2515 can2515;                  // CAN transport object

// Service objects
VLCB::LEDUserInterface ledUserInterface(LED_GRN, LED_YLW, SWITCH0);
VLCB::SerialUserInterface serialUserInterface;
VLCB::MinimumNodeServiceWithDiagnostics mnService;
VLCB::CanServiceWithDiagnostics canService(&can2515);
VLCB::NodeVariableService nvService;
VLCB::EventConsumerService ecService;
VLCB::EventTeachingService etService;
VLCB::EventProducerService epService;

long lastButtonPressTime = 0; // in ms.
byte possibleRoutes[NumEVs];

//
/// setup VLCB - runs once at power on from setup()
//
void setupVLCB()
{
  VLCB::checkStartupAction(LED_GRN, LED_YLW, SWITCH0);

  VLCB::setServices({
    &mnService, &ledUserInterface, &serialUserInterface, &canService, &nvService,
    &ecService, &epService, &etService});

  // set config layout parameters
  VLCB::setNumNodeVariables(10);
  VLCB::setMaxEvents(32);
  VLCB::setNumEventVariables(NumEVs);

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

bool isSubsequentButtonPress()
{
  // Check timer. Is this within the interval from the first button press?
  byte buttonPressInterval = VLCB::readNV(1) * 100;
  long now = millis();
  return now < lastButtonPressTime + buttonPressInterval;
}

void saveLastButtonPressTime()
{
  lastButtonPressTime = millis();
}

void saveRoutesFromEvent(byte eventIndex)
{
  // Save routes for this event for next button press.
  for (byte i = 0; i < NumEVs; i++)
  {
    possibleRoutes[i] = VLCB::getEventEVval(eventIndex, i + 1);
  }
}

bool routeIsSaved(byte newRoute)
{
  for (byte j = 0; j < NumEVs; j++)
  {
    if (possibleRoutes[j] == newRoute)
    {
      return true;
    }
  }
  return false;
}

byte findMatchingRoute(byte eventIndex)
{
  // Match route set with previous route set. (Intersection)
  byte selectedRoute = 0;
  byte routeCount = 0;
  for (byte i = 0; i < NumEVs; i++)
  {
    byte newRoute = VLCB::getEventEVval(eventIndex, i + 1);
    if (routeIsSaved(newRoute))
    {
      ++routeCount;
      if (selectedRoute == 0)
      {
        selectedRoute = newRoute;
      }
    }
  }
  if (routeCount == 0)
  {
    Serial << F("> No possible routes found.") << endl;
  }
  else
  {
    if (routeCount >= 2)
    {
      Serial << F("> Found ") << routeCount << F(" possible routes") << endl; 
    }
  }
  return selectedRoute;
}

//
/// user-defined event processing function
/// called from the VLCB library when a learned event is received
/// it receives the event table index and the CAN frame
//
void eventhandler(byte eventIndex, const VLCB::VlcbMessage *msg)
{
  Serial << F("> event handler: index = ") << eventIndex << F(", opcode = 0x") << _HEX(msg->data[0]) << endl;

  // Event Off op-codes have odd numbers.
  bool ison = (msg->data[0] & 0x01) == 0;
  if (!ison)
  {
    // Don't react to OFF events. Probably misconfiguration.
    return;
  }
  
  if (isSubsequentButtonPress())
  {
    Serial << F("> is subsequent button press") << endl;
    // Subsequent button press: Collect possible route set for this button.
    byte selectedRoute = findMatchingRoute(eventIndex);

    Serial << F("> Selected route ") << selectedRoute << endl;
    // Send an event with this node NN and the selectedRoute as EN. Cannot be taught.
    VLCB::sendMessageWithNN(OPC_ACON, 0, selectedRoute);
  }

  saveRoutesFromEvent(eventIndex);
  Serial << F("> saved routes from event for future button press.") << endl;
  saveLastButtonPressTime();
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
