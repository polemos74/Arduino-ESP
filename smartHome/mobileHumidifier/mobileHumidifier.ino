// Humidifer controller client

#include "CommonEssentials.h"

// SONOFF pin numbers
#define LED_BUILTIN 13
#define RELAY_PIN 12
#define INDICATOR_LED_PIN 14
#define PUSH_BUTTON  0            // Onboard push button
#define LED_ON LOW
#define LED_OFF HIGH

char clientName[] = {"mobileHumidifier"};

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
  uint16_t packetIdSub1 = mqttClient.subscribe("mobile/humidifier/command", 1);
  uint16_t packetIdSub2 = mqttClient.subscribe("mobile/humidifier/reset", 0);
  presenceReportTimer.attach(presenceReportInterval, presenceReportTimerCallback);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
}

void onMqttUnsubscribe(uint16_t packetId) {
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
//Serial.println(topic);
	if (String(topic) == "mobile/humidifier/command") {
    if (payload[1] == 'n')  {
    digitalWrite(RELAY_PIN,HIGH);
    digitalWrite(LED_BUILTIN,LED_OFF);
    digitalWrite(INDICATOR_LED_PIN,LED_ON);
	 mqttClient.publish("mobile/humidifier/feedback", 1, true, "on");
    }
    if (payload[1] == 'f'){
    digitalWrite(RELAY_PIN,LOW);
    digitalWrite(LED_BUILTIN,LED_ON);
    digitalWrite(INDICATOR_LED_PIN,LED_OFF);
	 mqttClient.publish("mobile/humidifier/feedback", 1, true, "off");
     }
	}
  if (String(topic) == "mobile/humidifier/reset") {
    ESP.restart();
  }
}

void onMqttPublish(uint16_t packetId) {
}

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LED_OFF);
  pinMode(INDICATOR_LED_PIN,OUTPUT);
  digitalWrite(INDICATOR_LED_PIN,LED_OFF);
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
  pinMode (PUSH_BUTTON,INPUT_PULLUP);

  attachInterrupt(PUSH_BUTTON,buttonPressed,FALLING);

  mqttPrefix = "mobile/humidifier";

  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);

  int buttonCheck = digitalRead(PUSH_BUTTON);
  if (buttonCheck == LOW) {
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
  mqttClient.setKeepAlive(keepAliveTimeout).setCleanSession(false).setWill("clients/mobile/humidifier/lastwill", 2, true, "disconnected");

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

void buttonEvent1() {
    digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));

    bool pinStatus = digitalRead(RELAY_PIN) ;
    if (pinStatus == HIGH){
      digitalWrite(LED_BUILTIN,LED_OFF);
      digitalWrite(INDICATOR_LED_PIN,LED_ON);
      mqttClient.publish("mobile/humidifier/pushButton", 1, true, "on");
    }
    else {
      digitalWrite(LED_BUILTIN,LED_ON);
      digitalWrite(INDICATOR_LED_PIN,LED_OFF);
      mqttClient.publish("mobile/humidifier/pushButton", 1, true, "off");
    }
}

void buttonEvent2()  {
  bool indicatorLedState = digitalRead(INDICATOR_LED_PIN);
  bool statusLedState = digitalRead(LED_BUILTIN);
  if (indicatorLedState == LED_ON && statusLedState == LED_OFF); digitalWrite(INDICATOR_LED_PIN,LED_OFF); bool indicateFlag = true;
  blinkerTimer.attach(0.15, blinker);
  delay(1000);
  blinkerTimer.detach();
  if (indicateFlag == true) {
    digitalWrite(INDICATOR_LED_PIN,LED_ON); digitalWrite(LED_BUILTIN,LED_OFF); indicateFlag = false;
  }
  else {
    digitalWrite(LED_BUILTIN,LED_ON);
  }
  mqttClient.publish("mobile/humidifier/pushButton", 1, true, "switch");
}
