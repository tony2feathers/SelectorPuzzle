//+------------------------------------------------------------------------
//
// Two Feathers LLC - (c) 2022 Robert Nelson. All Rights Reserved.
//
//File: main.cpp
//
// Description:
//
//      Program for controlling the Selector Puzzle portion of the marble run
//      Players must point the selector dial to the correct symbol for the marble to continue on the right path
//      Otherwise, the ball will return to the Vending Machine
//
// History:     NOV-18-2023       tony2feathers     Created
//              MAR-09-2024       tony2feathers     Added MQTT and WiFi functionality & created position arrays
//              MAR-10-2024       tony2feathers     Added stepper motor functionality, increased microstepping to 1600, recalculated positions,
//                                                  added recalibration function, implemented wifi and mqtt functionality
//                                                  Published to gitHub
//-------------------------------------------------------------------------

//INCLUDES
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_secrets.h"
#include <PubSubClient.h>
#include <AccelStepper.h>

// GLOBALS

  int lastPosition = 0;
  int currentPosition = 0;
  int selectedPosition = 0;
  int clockwiseSteps = 0;
  int counterClockwiseSteps = 0;
  int stepsToMove = 0;
  int selectionCounter = 0;

// Constants
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;  // the WiFi radio's status

const char* mqtt_server = "10.1.10.55";

WiFiClient espClient;
PubSubClient client(espClient);

// constants for MQTT
const char topic[] = "ToDevice/Selector";
const char hostTopic[] = "ToHost/Selector";
const char* deviceID = "Selector";

// Define pins for the Nema17 Stepper motor used for the SelectorBox
const byte SelectorDir = 14;
const byte SelectorStep = 26;
const byte SelectorEna = 27;
// Define motor interface type
#define motorInterfaceType 1

// Initiate the instance of the stepper motor
AccelStepper SelectorStepper = AccelStepper(motorInterfaceType, SelectorStep, SelectorDir);

#ifndef DEBUG
#define DEBUG
#endif

//Initialize Hall Sensor
const byte hallSensor = 32;

// define button pins for Selector Switch and create button array
const int SelectorNum = 8;

const int buttonMoonPin(22);  // Wire color: Purple
const int buttonStar5Pin(13);  // Wire color: White
const int buttonStar4Pin(4);  // Wire color: Orange
const int buttonTriCirclePin(23);  // Wire color: Grey
const int buttonCrossTrianglePin(33);  // Wire color: Yellow
const int buttonLightningPin(18);  // Wire color: Blue
const int buttonTopHatPin(19);  // Wire color: Red
const int buttonAirShipPin(21);  // Wire color: Green

// Create an array of INPUT_PULLUP pins for the selector knob
const int selectorArray[] = {
  buttonTriCirclePin,
  buttonStar5Pin,
  buttonAirShipPin,
  buttonLightningPin,
  buttonMoonPin,
  buttonCrossTrianglePin,
  buttonTopHatPin,
  buttonStar4Pin
  };

int stepsPerPosition = 200;

// Array of positions for each shape
int shapePositions[] = {
  575,
  375,
  175,
  1575,
  1375,
  1175,
  975,
  775
};

// Array of names of shapes
String shapeNames[] = {
  "Tri-Circle",       //144, add 1, 575
  "5 Pointed Star",   //95, add 1, 375
  "Air Ship",         //45, add 1, 175
  "Lightning",          //395,add 1, 1575
  "Moon",               //345, add 1, 1375
  "Cross and Triangle", //295, add 1, 1175
  "Top Hat",            //244, add 1, 975
   "4 Pointed Star"     //194, add 1, 775
};

// Variable to hold last selector knob value
int lastSelection = 8;

//***************FUNCTION DECLARATIONS***********************
void onSolve();
void onReset();
void stepperHome();
void selectShape(int shapeIndex);

//*********************INBOUND MQTT MESSAGE FUNCTIONS*********************
void callback(char* thisTopic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(thisTopic);
  Serial.print("] ");
  Serial.println("Message: ");


  // Convert byte array to C-style string
  char messageArrived[length + 1];
  memcpy(messageArrived, message, length);
  messageArrived[length] = '\0'; // Null-terminate the string
 
  for (unsigned int i = 0; i < length; i++) {
    messageArrived[i] = tolower(messageArrived[i]);
  }

  // Use strcmp to look for certain messages
  if (strcmp(messageArrived, "message") == 0){
    onSolve();

  #ifdef DEBUG
  Serial.println(messageArrived);
  Serial.println();
  #endif
  }
  else if (strcmp(messageArrived, "reset") == 0){
    onReset();
  }
  else {
    #ifdef DEBUG
    Serial.println("Message not recognized");
    #endif
  }
}


//************WIFI and MQTT FUNCTIONS************************

void wifiSetup() {
  Serial.println();
  Serial.println("****************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP Address: ");
  Serial.print(WiFi.localIP());
}



void MQTTsetup() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(topic);
  while (!client.connected()) {

    // Debug info
    Serial.print("Attempting to connect to the MQTT broker at ");
    Serial.println(mqtt_server);

    // Attempt to connect
    if (client.connect(deviceID)) {

      // Debug info
      Serial.println("Connected to MQTT broker");

      // Once connected, publish an announcement to the host
      client.publish(hostTopic, "Selector Connected!");
      // Subscribe to topics meant for this device
      client.subscribe(topic);
      Serial.println("Subscribed to topic: ");
      Serial.println(topic);
    } else {
      // Debug info
      Serial.print("Failed to connect to MQTT broker, rc =");
      Serial.print(client.state());
      Serial.println("Retrying in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}

//******************SETUP AND LOOP FUNCTIONS***********************
void setup() {
  //Initiate Serial Port for monitoring and debugging
  Serial.begin(9600);

  //Set pin modes
  pinMode(hallSensor, INPUT_PULLUP);

  // Establish Stepper Motor Settings
  SelectorStepper.setMaxSpeed(1000);
  SelectorStepper.setAcceleration(100);
  SelectorStepper.setSpeed(400);
  
  //Call Function to home Stepper Motor
  stepperHome();
  delay(1500);

  // Set the SelectorArray pins to INPUT_PULLUP
  for (int i = 0; i < SelectorNum; i++) {
    pinMode(selectorArray[i], INPUT_PULLUP);
  }

  // Move the stepper motor 25 steps to align with the first shape
  SelectorStepper.moveTo(175);
  SelectorStepper.setSpeed(400);

  while(SelectorStepper.distanceToGo() != 0){
    SelectorStepper.runSpeedToPosition();
  }

  // Set current and last position to 0
  lastPosition = 175;
  lastSelection = 2;
  currentPosition = 175;

  // Setup the wifi and MQTT connections
  wifiSetup();
  MQTTsetup();
  delay(500);
  Serial.print("Connected to WIFI and MQTT!");

  // Print messages to the serial monitor
  Serial.println("Position set to 1st symbol");
  Serial.println("Selector Puzzle Ready");
  delay(3000);

}

void loop() {
  #ifdef DEBUG
  Serial.println("Looping");
  #endif
  
  for(int i = 0; i < SelectorNum; i++){
    
    // If button is pressed, print a serial statement stating which button was pressed
    if (digitalRead(selectorArray[i]) == LOW) {
      #ifdef DEBUG
      Serial.print(shapeNames[i]);     
      Serial.println(" button pressed!");

      delay(500);
      #endif
      if (shapePositions[i] != lastPosition) {
        selectShape(i); // Increment the selection counter and re-calibrate the stepper motor if necessary
        Serial.println("New Position Selected, moving...");

        selectedPosition = shapePositions[i];

        int clockwiseSteps = (selectedPosition - SelectorStepper.currentPosition() + 1600) % 1600;
        int counterClockwiseSteps = ((SelectorStepper.currentPosition() - selectedPosition + 1600) % 1600);

        #ifdef DEBUG
        Serial.println("Clockwise Steps: " + String(clockwiseSteps));
        Serial.println("Counter Clockwise Steps: " + String(counterClockwiseSteps));
        #endif

        int stepsToMove = (clockwiseSteps < counterClockwiseSteps) ? clockwiseSteps : -counterClockwiseSteps;

        #ifdef DEBUG
        Serial.println("Steps to Move: " + String(stepsToMove));
        #endif

        SelectorStepper.move(stepsToMove);
        #ifdef DEBUG
        Serial.println("Moving to " + String(selectedPosition) + " with " + String(stepsToMove) + " steps");
        #endif
        SelectorStepper.setSpeed(400);
        while (SelectorStepper.distanceToGo() != 0) {
          SelectorStepper.runSpeedToPosition();
        }
        currentPosition = shapePositions[i];
        lastPosition = shapePositions[i];
      }
    }
    else{
      #ifdef DEBUG
      Serial.print(shapeNames[i]);       
      Serial.println(" button not pressed");
      #endif    
    }
    delay(500);
  }
  if (currentPosition == shapePositions[1]){
    onSolve();
  }

  #ifdef DEBUG
  Serial.print("Looping Complete. Pausing for 1 second");
  delay(1000);
  #endif  
}

void onSolve() {
  #ifdef DEBUG
  Serial.println("Selector Puzzle Solved!");
  #endif
  //client.publish(hostTopic, "SOLVED");
}

void onReset() {
  #ifdef DEBUG
  Serial.println("Selector Puzzle Reset!");
  #endif
  stepperHome();
  client.publish(hostTopic, "RESET");
  delay(1000);
  SelectorStepper.moveTo(stepsPerPosition / 2);
      SelectorStepper.setSpeed(200);
      while(SelectorStepper.distanceToGo() != 0){
        SelectorStepper.runSpeedToPosition();
      }
  lastSelection = 0;
  currentPosition = 0;

}

void stepperHome() {
    while(digitalRead(hallSensor) == 1){
     SelectorStepper.setSpeed(200);
     SelectorStepper.runSpeed();
  }
  SelectorStepper.setCurrentPosition(0);
  #ifdef DEBUG
  Serial.println("Stepper Homed");
  #endif
  //client.publish(hostTopic, "RESET");
}

void selectShape(int shapeIndex) {
  // Increment the counter each time a shape is selected
  selectionCounter++;
  #ifdef DEBUG
  Serial.println("Selection Counter: " + String(selectionCounter));
  #endif

  // If 10 selections have been made, re-calibrate the stepper motor and reset the counter
  if (selectionCounter == 10) {
    stepperHome();
    selectionCounter = 0;
    #ifdef DEBUG
    Serial.println("Stepper Motor Re-Calibrated and Selection Counter Reset");
    #endif
  }
}