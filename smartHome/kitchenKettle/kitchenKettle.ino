// Kitchen kettle client

#include "wifi_credentials.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

String mqttPrefix = "kitchen/kettle";

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883

// SONOFF pin numbers
//#define LED_BUILTIN 13
#define RELAY_PIN 12
#define INDICATOR_LED_PIN 14
#define PUSH_BUTTON  0            // Onboard push button

uint16_t time_elapsed = 0;
volatile bool buttonFlag = false ;
#define debounce 20 // ms debounce period to prevent flickering when pressing or releasing the button
#define holdTime 2000 // ms hold period: how long to wait for press+hold event

// Button variables
bool buttonVal = LOW; // value read from button
bool buttonLast = HIGH; // buffered value of the button's previous state
long btnDnTime; // time the button was pressed down
long btnUpTime; // time the button was released
boolean ignoreUp = false; // whether to ignore the button release because the click+hold was triggered

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

Ticker blinkerTimer;
Ticker timeoutCheckTimer;

char clientName[] = {"kitchenKettle"};
char kettleTimeoutBuffer [2];
int kettleTimeout = 5;
bool timeoutFlag = false ;
unsigned long  endTime ;

void initialConnectionForOTA()  {
  connectToWifi();
  OTA_setup();  
  flashTheLed(INDICATOR_LED_PIN);   // Flash the red LED 3 times

  while(time_elapsed < 30000) {
    ArduinoOTA.handle();
    time_elapsed = millis();
    delay(10);
  }   
  ESP.restart();
}

void connectToWifi() {
  // Connect to WiFi access point.  
  WiFi.begin(WLAN_SSID, WLAN_PASS);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  //Serial.println("Connected to Wi-Fi.");
  blinkerTimer.detach();
  digitalWrite(LED_BUILTIN, LOW);
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  //Serial.println("Disconnected from Wi-Fi.");
  blinkerTimer.attach(0.5, blinker);
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  //Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  //Serial.println("Connected to MQTT.");
  //Serial.print("Session present: ");
  //Serial.println(sessionPresent);
  uint16_t packetIdSub1 = mqttClient.subscribe("kitchen/kettle/command", 1);
  uint16_t packetIdSub2 = mqttClient.subscribe("kitchen/kettle/timeout", 2);
  uint16_t packetIdSub3 = mqttClient.subscribe("kitchen/kettle/reset", 0);
  mqttClient.publish("kitchen/kettle/availability", 1, true, "Online");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  //Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  //Serial.println("Subscribe acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
  //Serial.print("  qos: ");
  //Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  //Serial.println("Unsubscribe acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  //  Serial.println("Publish received.");
  //  Serial.print("  topic: ");
  //  Serial.println(topic);
  //  Serial.print("  payload: ");
  //  Serial.println(payload);  
  //  Serial.print("  qos: ");
  //  Serial.println(properties.qos);
  //  Serial.print("  dup: ");
  //  Serial.println(properties.dup);
  //  Serial.print("  retain: ");
  //  Serial.println(properties.retain);
  //  Serial.print("  len: ");
  //  Serial.println(len);
  //  Serial.print("  index: ");
  //  Serial.println(index);
  //  Serial.print("  total: ");
  //  Serial.println(total);
  //  Serial.println();    

  if (String(topic) == "kitchen/kettle/command") {
    if (payload[1] == 'n')  {
      digitalWrite(RELAY_PIN,HIGH);
      digitalWrite(LED_BUILTIN,HIGH);
      digitalWrite(INDICATOR_LED_PIN,LOW);
      timeoutCheckTimer.once(kettleTimeout,timeoutCheckTimerFunc);      
      mqttClient.publish("kitchen/kettle/feedback", 1, true, "on");
      // Serial.println("Publishing at QoS 1");      
    } 
    if (payload[1] == 'f')  {
      digitalWrite(RELAY_PIN,LOW);
      digitalWrite(LED_BUILTIN,LOW);
      digitalWrite(INDICATOR_LED_PIN,HIGH);    
      timeoutCheckTimer.detach();
      mqttClient.publish("kitchen/kettle/feedback", 1, true, "off");
      // Serial.println("Publishing at QoS 1");      
    }
  }
  if (String(topic) == "kitchen/kettle/timeout") {
    strncpy (kettleTimeoutBuffer,payload,2);
    kettleTimeout = atoi(kettleTimeoutBuffer);
  }   
  if (String(topic) == "kitchen/kettle/reset") {
    delay(2000);
    ESP.restart();
  }  
}

void onMqttPublish(uint16_t packetId) {
  //Serial.println("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  pinMode(INDICATOR_LED_PIN,OUTPUT);
  digitalWrite(INDICATOR_LED_PIN,HIGH);
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
  pinMode (PUSH_BUTTON,INPUT_PULLUP);

  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);

  int buttonCheck = digitalRead(PUSH_BUTTON);
    if(buttonCheck == LOW){
      initialConnectionForOTA();
    }
  
  attachInterrupt(PUSH_BUTTON,buttonPressed,FALLING);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.setClientId(clientName);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setKeepAlive(5).setCleanSession(false).setWill("kitchen/kettle/lastwill", 2, true, "disconnected");

  blinkerTimer.attach(0.5, blinker);

  connectToWifi();

  buttonFlag = false;

//  WiFi.printDiag(Serial);

}

void loop() {
  checkButton();
  checkTimeout();
}
void timeoutCheckTimerFunc() {
  timeoutFlag = true ;
}

void checkTimeout() {
  if (timeoutFlag == true) {
    buttonEvent1() ;
//    timeoutCheckTimer.detach();
    timeoutFlag = false;
  }
}

void checkButton()  {
  if (buttonFlag == true) {
    //Serial.println("Button pressed!");
    ////Serial.println(pinStatus);
    buttonVal = digitalRead(PUSH_BUTTON);
    // Test for button pressed and store the down time
    if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnUpTime) > long(debounce))  {
      btnDnTime = millis();
      //Serial.println("Down "+String(btnDnTime));
    }
    // Test for button release and store the up time
    if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnDnTime) > long(debounce))  {
      if (ignoreUp == false) buttonEvent1();
      else ignoreUp = false;
      btnUpTime = millis();
      //Serial.println("Up "+String(btnUpTime));
      buttonFlag = false;
    }
    // Test for button held down for longer than the hold time
    if (buttonVal == LOW && (millis() - btnDnTime) > long(holdTime))  {
      buttonEvent2();
      ignoreUp = true;
      btnDnTime = millis();
      //Serial.println("Down "+String(btnDnTime));
    }
    buttonLast = buttonVal;
  }
}

void buttonPressed()  {
  buttonFlag = true;
}

void buttonEvent1() {
    digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
    digitalWrite(INDICATOR_LED_PIN, !digitalRead(INDICATOR_LED_PIN));
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    bool pinStatus = digitalRead(RELAY_PIN) ;
    if (pinStatus == HIGH){
      timeoutCheckTimer.once(kettleTimeout,timeoutCheckTimerFunc);      
      mqttClient.publish("kitchen/kettle/feedback", 1, true, "on");
    }
    else {
      timeoutCheckTimer.detach();
      mqttClient.publish("kitchen/kettle/feedback", 1, true, "off");
    }
}

void buttonEvent2() {
  if (digitalRead(INDICATOR_LED_PIN) == LOW) digitalWrite(INDICATOR_LED_PIN,HIGH);
  flashTheLed(LED_BUILTIN); // Flash the green LED 3 times
  uint16_t packetIdPub1 = mqttClient.publish("kitchen/kettle/pushButton", 1, true, "off");
  delay(2000);
  ESP.restart();
}

void OTA_setup()  {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("OTA_test");

  // No authentication by default   ArduinoOTA.setPassword("bsb");

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

void flashTheLed(int ledToFlash){
  for (int a = 1 ; a < 7 ; a++){
    digitalWrite(ledToFlash,!digitalRead(ledToFlash));
    delay(100);
  }
}

void blinker() {
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}
