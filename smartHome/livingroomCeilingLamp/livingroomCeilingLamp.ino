// Living room ceiling lamp client

#include "CommonEssentials.h"

#define RELAY_PIN 12              // Onboard Relay
//#define SENSOR01_INPUT_PIN    5  // PIR sensor
#define PUSH_BUTTON  0            // Onboard push button
#define LED_ON LOW
#define LED_OFF HIGH

char clientName[] = {"livingCeilingLamp"};

void initialConnectionForOTA(){
  blinkerTimer.attach(0.15, blinker);
  connectToWifi();
  OTA_setup();

  uint16_t time_elapsed = 0;
  while(time_elapsed < 60000) {
    ArduinoOTA.handle();
    time_elapsed = millis();
    delay(10);
    }
  ESP.restart();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  blinkerTimer.detach();
  digitalWrite(LED_BUILTIN, LED_ON);
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  blinkerTimer.attach(0.5, blinker);
  mqttClient.disconnect();
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  uint16_t packetIdSub1 = mqttClient.subscribe("living/lamp/ceiling/command", 1);
  uint16_t packetIdSub2 = mqttClient.subscribe("living/lamp/ceiling/reset", 0);
  mqttClient.publish("clients/living/lamp/ceiling/greeting", 1, true, "connected");
  presenceReportTimer.attach(presenceReportInterval, presenceReportTimerCallback);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
}

void onMqttUnsubscribe(uint16_t packetId) {
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (String(topic) == "living/lamp/ceiling/command") {
    if (payload[1] == 'n')  {
      digitalWrite(RELAY_PIN, HIGH);
      mqttClient.publish("living/lamp/ceiling/feedback", 1, true, "on");
      }
    if (payload[1] == 'f'){
      digitalWrite(RELAY_PIN, LOW);
      mqttClient.publish("living/lamp/ceiling/feedback", 1, true, "off");
      }
    }
  if (String(topic) == "living/lamp/ceiling/reset") {
    ESP.restart();
  }
}

void onMqttPublish(uint16_t packetId) {
}

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LED_OFF);
  pinMode (RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
  pinMode (PUSH_BUTTON,INPUT_PULLUP);

  mqttPrefix = "living/lamp/ceiling";

  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);

  int buttonCheck = digitalRead(PUSH_BUTTON);
  if(buttonCheck == LOW){
    initialConnectionForOTA();
  }

  attachInterrupt(PUSH_BUTTON,buttonPressed,FALLING);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setKeepAlive(keepAliveTimeout).setCleanSession(false).setWill("clients/living/lamp/ceiling/lastwill", 2, true, "disconnected");

  blinkerTimer.attach(0.5, blinker);

  connectToWifi();

}

void loop() {
  checkButton();
  checkPresenceReportFlag();
}

void buttonPressed()  {
  buttonFlag = true;
}

void checkButton()  {
  if (buttonFlag == true) {
    //Serial.println("Button pressed!");
    buttonVal = digitalRead(PUSH_BUTTON);
    // Test for button pressed and store the down time
    if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnUpTime) > long(debounce))  {
      btnDnTime = millis();
      //Serial.println("Down "+String(btnDnTime));
    }
    // Test for button release and store the up time
    if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnDnTime) > long(debounce))  {
      if (ignoreUp == false) buttonEvent1();
      else ignoreUp = false; eventFired = false;
      btnUpTime = millis();
      //Serial.println("Up "+String(btnUpTime));
      buttonFlag = false;
    }
    // Test for button held down for longer than the hold time
    if (buttonVal == LOW && (millis() - btnDnTime) > long(holdTime) && eventFired == false)  {
      buttonEvent2();
      eventFired = true;
      ignoreUp = true;
      btnDnTime = millis();
      //Serial.println("Down "+String(btnDnTime));
    }
    buttonLast = buttonVal;
  }
}

void buttonEvent1() {
    digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));

    bool pinStatus = digitalRead(RELAY_PIN) ;
    if (pinStatus == HIGH){
      mqttClient.publish("living/lamp/ceiling/pushButton", 1, true, "on");
    }
    else {
      mqttClient.publish("living/lamp/ceiling/pushButton", 1, true, "off");
    }
}

void buttonEvent2()  {
  blinkerTimer.attach(0.15, blinker);
  blinkerStopper.once(1,blinkerStopperCallback);
  mqttClient.publish("living/lamp/ceiling/pushButton", 1, true, "switch");
}
