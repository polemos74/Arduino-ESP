// MQTT test client
// deneme 3

#include "TM1637Header.h"
#include "CommonEssentials.h"

// Wemos D1 mini pin numbers
#define PUSH_BUTTON D5
#define LED_ON LOW
#define LED_OFF HIGH

Ticker displayTimer;
//Ticker DHTReadTimer;

bool displayFlag  = false;
//bool DHTReadFlag = false;

char clientName[] = {"mqttTest"};

void initialConnectionForOTA() {
  blinkerTimer.attach(0.15, blinker);
  display.print("OTA ");
  connectToWifi();
  OTA_setup();

  uint16_t time_elapsed = 0;
  while (time_elapsed < 60000) {
    ArduinoOTA.handle();
    time_elapsed = millis();
    delay(10);
  }
  ESP.restart();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  display.print("WL C");
  blinkerTimer.detach();
  digitalWrite(LED_BUILTIN, LED_OFF);
  connectToMqtt();
}
void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  display.print("WL D");
  blinkerTimer.attach(0.5, blinker);
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  display.print("MQ C");
  uint16_t packetIdSub = mqttClient.subscribe("common/timestamp", 1);
  presenceReportTimer.attach(presenceReportInterval, presenceReportTimerCallback);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
}

void onMqttUnsubscribe(uint16_t packetId) {
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (String(topic) == "common/timestamp") {
    if (len < 10) {
      return;
    }
    char timestampBuffer[10];   // Temporary buffer
    strncpy (timestampBuffer, payload, 10);
    timestamp = atoi(timestampBuffer);
    timestampReceived = true;
  }
}

void onMqttPublish(uint16_t packetId) {
}

void setup() {

  //  Serial.begin(115200);
  display.begin();            // initializes the display
  display.setBacklight(100);  // set the brightness to 100 %
  display.print("INIT");      // display INIT on the display

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_OFF);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);

  mqttPrefix = "mqttTest";

  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);

  int buttonCheck = digitalRead(PUSH_BUTTON);
  if (buttonCheck == LOW) {
    initialConnectionForOTA();
  }

  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON), buttonPressed, FALLING);

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
  mqttClient.setKeepAlive(keepAliveTimeout).setCleanSession(false).setWill("clients/mqttTest/lastwill", 2, true, "disconnected");

  blinkerTimer.attach(0.5, blinker);

  display.print("C WL");

  connectToWifi();

  while (!timestampReceived) {
    yield();     //Serial.println("no packet yet");
  }
  timeSystemInit();

  displayTimer.attach(2, displayTimerCallback);
  //  DHTReadTimer.attach(10,DHTReadTimerCallback);
}

void loop() {
  checkButton();
  checkClockSync();
  //  checkDHTReadTimerFlag();
  checkDisplayFlag();
  checkPresenceReportFlag();
}


void buttonPressed() {
  buttonFlag = true;
}

void checkButton() {
  if (buttonFlag == true) {
    buttonVal = digitalRead(PUSH_BUTTON);
    // Test for button pressed and store the down time
    if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnUpTime) > long(debounce))  {
      btnDnTime = millis();
    }
    // Test for button release and store the up time
    if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnDnTime) > long(debounce))  {
      if (ignoreUp == false) event1();
      else ignoreUp = false;
      btnUpTime = millis();
      buttonFlag = false;
    }
    // Test for button held down for longer than the hold time
    if (buttonVal == LOW && (millis() - btnDnTime) > long(holdTime))  {
      event2();
      ignoreUp = true;
      btnDnTime = millis();
    }
    buttonLast = buttonVal;
  }
}

void event1() {
  for (int a = 1 ; a < 2 ; a++) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void event2()  {
  for (int a = 1 ; a < 4 ; a++) {
    digitalWrite(LED_BUILTIN, LED_ON);
    delay(100);
    digitalWrite(LED_BUILTIN, LED_OFF);
    delay(100);
  }
}

void displayTimerCallback() {
  displayFlag = true;
}

void checkDisplayFlag() {
  if (displayFlag == true) {
    switch (displayData) {
      case  0:
        display.clear();
        display.printTime(hour(), minute(), false);
        break;

      case  1:
        //    numberToDisplay = int(temperature);
        display.clear();
        display.print(" 00");
        display.printRaw(B01100011, 3); //degree sign
        break;

      case  2:
        //    numberToDisplay = int(humidity);
        display.clear();
        display.print("H 00");

    }
    displayData++;
    if (displayData > 2) {
      displayData = 0;
    }
    displayFlag = false;
  }
}
