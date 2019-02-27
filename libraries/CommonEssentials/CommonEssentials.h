#include "wifi_credentials.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <TimeLib.h>

volatile bool buttonFlag = false ;
#define debounce 20 // ms debounce period to prevent flickering when pressing or releasing the button
#define holdTime 1000 // ms hold period: how long to wait for press+hold event

// Button variables
bool buttonVal ; // value read from button
bool buttonLast = HIGH; // buffered value of the button's previous state
long btnDnTime; // time the button was pressed down
long btnUpTime; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered

#define TIMEZONE_OFFSET 3    // GMT+3 for Turkey

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883
String mqttPrefix;

unsigned long timestamp;    // Received timestamp from MQTT broker
boolean timestampReceived = false;  // Receive flag

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
const byte presenceReportInterval = 15;
const byte keepAliveTimeout = 15;

bool presenceReportTimerFlag = false;
Ticker presenceReportTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

bool clockSyncFlag = false;
Ticker blinkerTimer;
Ticker blinkerStopper;
Ticker clockSyncTimer;

void connectToWifi() {
  WiFi.begin(WLAN_SSID, WLAN_PASS);
}

void connectToMqtt() {
//  display.print("C MQ");
Serial.println("MQTT Connecting");
  mqttClient.connect();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("MQTT Disconnected");
//  display.print("MQ D");
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void setTheClock() {
  timestamp = timestamp + TIMEZONE_OFFSET * 60 * 60 ;
  setTime(timestamp);
}

void clockSyncTimerCallback() {
  clockSyncFlag = true;
}

void checkClockSync() {
  if (clockSyncFlag == true ) {
    setTheClock();
    clockSyncFlag = false;
  }
}

void presenceReportTimerCallback() {
  presenceReportTimerFlag = true;
}

void checkPresenceReportFlag() {
  if ( presenceReportTimerFlag == true ) {
    String topic = "clients/"+mqttPrefix+"/greeting";
    char topicChar[100];
    topic.toCharArray(topicChar,100);
    mqttClient.publish(topicChar, 1, true, "connected");
    presenceReportTimerFlag = false;
  }
}

void timeSystemInit() {
  setTheClock();
  clockSyncTimer.attach(3600, clockSyncTimerCallback);

  String bootTime = ((String)hour() + ":" + (String)minute() + ":" + (String)second());
  char bootTimeChar [8];
  bootTime.toCharArray(bootTimeChar, 8);

  String topic = "clients/"+mqttPrefix+"/bootTime";
  char topicChar[100];
  topic.toCharArray(topicChar,100);

  mqttClient.publish(topicChar, 1, true, bootTimeChar);
}

void blinker() {
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}

void OTA_setup() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("OTA_test");

  // No authentication by default
  ArduinoOTA.setPassword("bsb");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    //Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    //Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR); //Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR); //Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR); //Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR); //Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR); //Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
