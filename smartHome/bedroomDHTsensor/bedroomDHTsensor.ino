#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <TM1637Header.h>
#include <DHTheader.h>
#include "MQTTTimeSync.h"

#define DHT_PIN D4     // what pin we're connected to
//unsigned long previousMillis;

#include "wifi_credentials.h"

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883

// Wemos D1 mini pin numbers
//#define LED_BUILTIN D4
//#define SENSOR01_OUTPUT_PIN  D7  // DHT sensor


AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

char valueStr01[5];
char valueStr02[5];
char clientName[] = {"bedroomDHTSensor"};

unsigned long previousMillis;

void connectToWifi() {
    // Connect to WiFi access point.
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(WLAN_SSID);
  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is active low on the ESP-01)
    yield();
    delay(50);                      // Wait for a second
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    for (int a = 1 ; a < 10 ; a++){
    yield();
    delay(50);
    }

  }
  //Serial.println();

  //Serial.println("WiFi connected");
  //Serial.println("IP address: "); //Serial.println(WiFi.localIP());
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  //Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  //Serial.println("Disconnected from Wi-Fi.");
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
  uint16_t packetIdSub = mqttClient.subscribe("common/timestamp", 1);
/*  uint16_t packetIdSub = mqttClient.subscribe("son/humidifier/command", 1);
  //Serial.print("Subscribing at QoS 1, packetId: ");
  //Serial.println(packetIdSub);
  uint16_t packetIdSub = mqttClient.subscribe("living/pir/command", 1);
  //Serial.print("Subscribing at QoS 1, packetId: ");
  //Serial.println(packetIdSub);
    uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  //Serial.print("Publishing at QoS 1, packetId: ");
  //Serial.println(packetIdPub1);
  uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  //Serial.print("Publishing at QoS 2, packetId: ");
  //Serial.println(packetIdPub2);*/
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
//Serial.println(topic);
  if (String(topic) == "common/timestamp") {
        if (len < 13){
      return;
      }

//    Serial.println();      
//    Serial.println(payload);      
    strncpy (timestampBuffer,payload,10);
//    Serial.println(timestampBuffer);      
    timestamp = atoi(timestampBuffer);
//    Serial.println(timestamp);
    timeIsSet = true;
    } 
}

void onMqttPublish(uint16_t packetId) {
  //Serial.println("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  //Serial.begin(115200);
  display.begin();            // initializes the display
  display.setBacklight(100);  // set the brightness to 100 %
  display.print("INIT");      // display INIT on the display

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
  mqttClient.setKeepAlive(60).setCleanSession(false).setWill("bedroom/sensor/lastwill", 2, true, "disconnected");

  connectToWifi();

  for (int a = 1 ; a < 10 ; a++){
    yield();
    delay(50);
    }

  while (!timeIsSet) {
        yield();
    //Serial.println("no packet yet");
  }

  setTheClock();
  extractUnixTime();
  lastHourCheck = clockHour;

//  WiFi.printDiag(//Serial);

}

void loop() {

  unsigned long currentMillis = millis();
  epoch = epoch + (int((currentMillis-previousMillis)/1000));

  if (currentMillis - previousMillis >= 1000){
  previousMillis = currentMillis;
  extractUnixTime();
//      //Serial.println(lastHourCheck);
//      //Serial.println(clockHour);
  if (lastHourCheck != clockHour){
      setTheClock();
      extractUnixTime();
      lastHourCheck = clockHour;
      }
    if (clockSecond%5 == 0 ){
      readDHT();
      String hi01 = (String)temperature;
      String hi02 = (String)humidity;
      hi01.toCharArray(valueStr01, 5);
      hi02.toCharArray(valueStr02, 5);
    uint16_t packetIdPub1 = mqttClient.publish("bedroom/sensor/temperature", 0, true, valueStr01);
    uint16_t packetIdPub2 = mqttClient.publish("bedroom/sensor/humidity", 0, true, valueStr02);

      }
    if (clockSecond%2 == 0 ){
      switch (mode) {
        case  0:
        display.clear();
        display.printTime(clockHour,clockMinute,false);
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

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  /*
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  */
        }
      mode++;
      if (mode > 2){
        mode = 0;
        }

      }
    }
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

void setTheClock(){
    epoch = timestamp;
}


void extractUnixTime(){
  now = epoch;
      clockSecond = now.second();
      clockMinute = now.minute();
      clockHour = now.hour()+TIMEZONE_OFFSET;
      if (clockHour > 23){
      clockHour = clockHour - 24;
      }
  
  }

