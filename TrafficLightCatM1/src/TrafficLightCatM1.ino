/* --------------------------------------------------------------------------------------------
 * Intelligent Traffic Light management through LTE Cat M1
 * Created by Manuel Montenegro, Sep 23, 2019.
 * Developed for MOTAM project.
 * 
 *  This sketch manages Intelligent Traffic Light configuration messages such starting Traffic
 *  Light emergency protocol or changing the behaviour of traffic light cycle.
 *  
 *  Compatible boards: Particle Boron LTE.
-------------------------------------------------------------------------------------------- */

// ------------ Libraries ------------

#include "Arduino.h"
#include "ArduinoJson.h"

// ------------ Global variables ------------

// The default state of Intelligent Traffic Light is normal cycle
static const String defaultConf = "{\"mode\":\"dynamic\", \"specificData\":{\"redTime\":30,\"yellowTime\":6,\"greenTime\":30}}";

// DynamicJsonDocument for managing state and string for receiving/send the state
String currentStateString;
DynamicJsonDocument currentStateJson (1024);

// Battery indicator
FuelGauge fuel;

// Timer for updating ITL current state
Timer currentStateTimer (1000, updateStateTimer);

// BLE data sent to nRF52840 (state + timeleft + timestamp)
uint8_t bleFrame [6];

// ------------ Functions ------------

// Cloud function. Change ITL configuration received from the cloud
int changeConf (String inputJson)
{
  Particle.publish("newConf",inputJson , PUBLIC);
  DynamicJsonDocument newConfJson (1024);
  DeserializationError err = deserializeJson(newConfJson, inputJson);
  if (!err) {
    currentStateJson["mode"] = newConfJson["mode"];
    currentStateJson["specificData"] = newConfJson["specificData"];
    currentStateString="";
    serializeJson(currentStateJson, currentStateString);

    return 0;
  }
  else
  {
    return -1;
  }
}

// Timer handler
void updateStateTimer ( )
{
  if (currentStateJson["mode"]=="dynamic")
  {
    int timeout = currentStateJson["timeout"];
    currentStateJson["timeout"] = (timeout - 1);
    if (timeout == 0)                            // Changing the state (color)
    {
      int state = currentStateJson["state"];
      if (state == 0)
      {
        state = 2;
      }
      else
      {
        state --;
      }
      currentStateJson["state"] = state;

      // Fitting the new timeout
      if (state == 2)                           // Green state
      {
        currentStateJson["timeout"] = currentStateJson["specificData"]["greenTime"];
      }
      if (state == 1)                           // Yellow state
      {
        currentStateJson["timeout"] = currentStateJson["specificData"]["yellowTime"];
      }
      if (state == 0)                           // Red state
      {
        currentStateJson["timeout"] = currentStateJson["specificData"]["redTime"];
      }
    }
    bleFrame[1] = currentStateJson["timeout"];
  }
  else if (currentStateJson["mode"]=="static")
  {
    currentStateJson["state"] = currentStateJson["specificData"]["state"];
    bleFrame[1] = 0xFF;
  }

  currentStateJson["battery"] = fuel.getSoC();

  currentStateString = "";
  serializeJson(currentStateJson, currentStateString);
  Serial.println(currentStateString);

  // BLE data
  bleFrame[0] = currentStateJson["state"];
  time_t timestamp = Time.now();
  memcpy(&bleFrame[2], &timestamp, sizeof(timestamp));

  // Send BLE data to nRF52840
  Serial1.write(bleFrame, sizeof(bleFrame));
}


void setup ()
{
  Particle.keepAlive(30);

  Serial.begin(115200);
  Serial1.begin(115200);

  // Current Intelligent Traffic Light state
  deserializeJson(currentStateJson, defaultConf);
  currentStateJson["battery"] = fuel.getSoC();
  currentStateJson["state"] = 2;
  currentStateJson["timeout"] = currentStateJson["specificData"]["greenTime"];
  serializeJson(currentStateJson, currentStateString);

  // BLE data
  bleFrame[0] = currentStateJson["state"];
  bleFrame[1] = currentStateJson["timeout"];
  time_t timestamp = Time.now();
  memcpy(&bleFrame[2], &timestamp, sizeof(timestamp));

  // Send BLE data to nRF52840
  Serial1.write(bleFrame, sizeof(bleFrame));

  // Cloud variables
  Particle.variable ("state", currentStateString);

  // Cloud variables
  Particle.function("changeConf",changeConf);

  currentStateTimer.start();
}


void loop ()
{

}