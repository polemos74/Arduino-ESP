// Bedroom DHT sensor with display client

#include <TM1637Header.h>
#include "CommonEssentials.h"
#include <DHTheader.h>

#define DHT_PIN D4     // what pin we're connected to
#define SWITCH_PIN D5              // Onboard Transistor Switch
#define PUSH_BUTTON D6
#define LED_ON LOW
#define LED_OFF HIGH

char clientName[] = {"sonDHTSensor"};

Ticker displayTimer;
Ticker DHTReadTimer;

bool displayFlag  = false;
bool DHTReadFlag = false;

void initialConnectionForOTA(){
  blinkerTimer.attach(0.15, blinker);
  display.print("OTA ");
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
  digitalWrite(LED_BUILTIN, LED_OFF);
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  blinkerTimer.attach(0.5, blinker);
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  uint16_t packetIdSub = mqttClient.subscribe("common/timestamp", 1);
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
}

void onMqttUnsubscribe(uint16_t packetId) {
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (String(topic) == "common/timestamp") {
    if (len < 10){
    return;
    }
    char timestampBuffer[10];   // Temporary buffer
    strncpy (timestampBuffer,payload,10);
    timestamp = atoi(timestampBuffer);
    timestampReceived = true;
  }
}

void onMqttPublish(uint16_t packetId) {
}

void setup() {
  display.begin();            // initializes the display
  display.setBacklight(100);  // set the brightness to 100 %
  display.print("INIT");      // display INIT on the display

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LED_OFF);
  pinMode(SWITCH_PIN,OUTPUT);
  digitalWrite(SWITCH_PIN,LOW);
  pinMode(PUSH_BUTTON,INPUT_PULLUP);

  mqttPrefix = "son/sensor";

  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);

  int buttonCheck = digitalRead(PUSH_BUTTON);
  if(buttonCheck == LOW){
    initialConnectionForOTA();
  }

  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON),buttonPressed,FALLING);

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
  mqttClient.setKeepAlive(keepAliveTimeout).setCleanSession(false).setWill("clients/son/sensor/lastwill", 2, true, "disconnected");


  blinkerTimer.attach(0.5, blinker);

  connectToWifi();

  while (!timestampReceived) {
    yield();
  }
  timeSystemInit();

  displayTimer.attach(2, displayTimerCallback);
  DHTReadTimer.attach(10,DHTReadTimerCallback);
}

void loop() {
  checkButton();
  checkClockSync();
  checkDHTReadFlag();
  checkDisplayFlag();
  checkPresenceReportFlag();
}

void readDHT(){
  // READ DATA
    //Serial.print("DHT22, \t");
//    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because
                                    // it is active low on the ESP-01)
//    delay(50);                      // Wait for a second
//    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

    uint32_t start = micros();
    int chk = DHT.read22(DHT_PIN);
    uint32_t stop = micros();

    stat.total++;
    switch (chk)
    {
    case DHTLIB_OK:
        stat.ok++;
        //Serial.print("OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        stat.crc_error++;
        //Serial.print("Checksum error,\t");
        break;
    case DHTLIB_ERROR_TIMEOUT:
        stat.time_out++;
        //Serial.print("Time out error,\t");
        break;
    default:
        stat.unknown++;
        //Serial.print("Unknown error,\t");
        break;
    }
    // DISPLAY DATA
    humidity=DHT.humidity;
    //Serial.print(humidity, 1);
    //Serial.print(",\t");
    temperature=DHT.temperature;
    //Serial.print(temperature, 1);
    //Serial.print(",\t");
    //Serial.print(stop - start);
    //Serial.println();

    if (stat.total % 20 == 0)
    {
        //Serial.println("\nTOT\tOK\tCRC\tTO\tUNK");
        //Serial.print(stat.total);
        //Serial.print("\t");
        //Serial.print(stat.ok);
        //Serial.print("\t");
        //Serial.print(stat.crc_error);
        //Serial.print("\t");
        //Serial.print(stat.time_out);
        //Serial.print("\t");
        //Serial.print(stat.connect);
        //Serial.print("\t");
        //Serial.print(stat.ack_l);
        //Serial.print("\t");
        //Serial.print(stat.ack_h);
        //Serial.print("\t");
        //Serial.print(stat.unknown);
        //Serial.println("\n");
    }
  }

  void buttonPressed(){
    buttonFlag = true;
  }

  void checkButton(){
    if (buttonFlag == true) {
      //Serial.println("Handling interrupt");
      buttonVal = digitalRead(PUSH_BUTTON);
      // Test for button pressed and store the down time
      if (buttonVal == LOW && buttonLast == HIGH && (millis() - btnUpTime) > long(debounce))  {
        btnDnTime = millis();
        //Serial.println("Down "+String(btnDnTime));
      }
      // Test for button release and store the up time
      if (buttonVal == HIGH && buttonLast == LOW && (millis() - btnDnTime) > long(debounce))  {
        if (ignoreUp == false) event1();
        else ignoreUp = false;
        btnUpTime = millis();
        //Serial.println("Up "+String(btnUpTime));
        buttonFlag = false;
      }
      // Test for button held down for longer than the hold time
      if (buttonVal == LOW && (millis() - btnDnTime) > long(holdTime))  {
        event2();
        ignoreUp = true;
        btnDnTime = millis();
      //Serial.println("Down "+String(btnDnTime));
      }
      buttonLast = buttonVal;
    }
  }

  void event1() {
    digitalWrite(SWITCH_PIN,!digitalRead(SWITCH_PIN));
    if (digitalRead(SWITCH_PIN) == HIGH)  {
      mqttClient.publish("son/sensor/pushButton", 1, true, "on");
      //Serial.println("Publishing at QoS 1");
      }
    else {
      mqttClient.publish("son/sensor/pushButton", 1, true, "off");
      //Serial.println("Publishing at QoS 1");
    }
  }

  void event2()  {
    mqttClient.publish("son/sensor/pushButton", 1, true, "auto");
    for (int a = 1 ; a < 7 ; a++) {
      digitalWrite(SWITCH_PIN,!digitalRead(SWITCH_PIN));
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
          display.printTime(hour(),minute(),false);
          break;

          case  1:
          numberToDisplay = int(temperature);
          display.clear();
          display.print(" "+String(numberToDisplay));
          display.printRaw(B01100011,3);  //degree sign
          break;

          case  2:
          numberToDisplay = int(humidity);
          display.clear();
          display.print("H "+String(numberToDisplay));

        }
        displayData++;
        if (displayData > 2){
          displayData = 0;
        }
        displayFlag = false;
    }
  }

  void DHTReadTimerCallback() {
    DHTReadFlag = true;
    }

  void checkDHTReadFlag() {
    if ( DHTReadFlag == true ) {
        readDHT();
        char valueStr01[5];
        char valueStr02[5];
        String hi01 = (String)temperature;
        String hi02 = (String)humidity;
        hi01.toCharArray(valueStr01, 5);
        hi02.toCharArray(valueStr02, 5);
        uint16_t packetIdPub1 = mqttClient.publish("son/sensor/temperature", 0, true, valueStr01);
        uint16_t packetIdPub2 = mqttClient.publish("son/sensor/humidity", 0, true, valueStr02);
        DHTReadFlag = false;
    }
  }
