// Outside sensor client

#include "wifi_credentials.h"
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <DHTheader.h>

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883

// Wemos D1 mini pin numbers
#define DHT_PIN D4     // what pin we're connected to
#define LUM_POWER_PIN D5
#define DHT_MODEL 22

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

Ticker blinkerTimer;

char valueStr01[5];
char valueStr02[5];
char valueStr03[5];

char clientName[] = {"outsideDHTSensor"};

unsigned long previousMillis;

void connectToWifi() {
  // Connect to WiFi access point.
  //Serial.println(); 
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    yield();
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
//  uint16_t packetIdSub = mqttClient.subscribe("son/humidifier/command", 1);
//  //Serial.print("Subscribing at QoS 1, packetId: ");
//  //Serial.println(packetIdSub);
/*  uint16_t packetIdSub = mqttClient.subscribe("living/pir/command", 1);
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
/*if (String(topic) == "son/humidifier/command") {
    if (payload[0] == '1')  {
//      digitalWrite(LED_BUILTIN, LOW);
mqttClient.publish("son/humidifier/feedback", 2, true, "true");
  //Serial.println("Publishing at QoS 2");      
      } 
     else{
//      digitalWrite(LED_BUILTIN, HIGH);
mqttClient.publish("son/humidifier/feedback", 2, true, "false");
  //Serial.println("Publishing at QoS 2");      

     }
}*/
  //Serial.println("Publish received.");
  //Serial.print("  topic: ");
  //Serial.println(topic);
  //Serial.print("  payload: ");
  //Serial.println(payload);  
  //Serial.print("  qos: ");
  //Serial.println(properties.qos);
  //Serial.print("  dup: ");
  //Serial.println(properties.dup);
  //Serial.print("  retain: ");
  //Serial.println(properties.retain);
  //Serial.print("  len: ");
  //Serial.println(len);
  //Serial.print("  index: ");
  //Serial.println(index);
  //Serial.print("  total: ");
  //Serial.println(total);
  //Serial.println();    
}

void onMqttPublish(uint16_t packetId) {
  //Serial.println("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void setup() {

 pinMode(LED_BUILTIN,OUTPUT);
 digitalWrite(LED_BUILTIN,HIGH);
 pinMode(LUM_POWER_PIN,OUTPUT);
 digitalWrite(LUM_POWER_PIN,LOW);
 
  Serial.begin(115200);
  //Serial.println();
  //Serial.println();

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
  mqttClient.setKeepAlive(60).setCleanSession(false).setWill("balcony/sensor/lastwill", 2, true, "disconnected");


  WiFi.hostname(clientName);
  WiFi.mode(WIFI_STA);
  
  blinkerTimer.attach(0.5, blinker);

  connectToWifi();

  blinkerTimer.detach();

//  WiFi.printDiag(//Serial);

}

void loop() {
//  //Serial.println(analogRead(A0));
unsigned long currentMillis = millis();
if (currentMillis - previousMillis > 5000){
 previousMillis = currentMillis;
      digitalWrite(LUM_POWER_PIN,HIGH);
      unsigned int  luminosity = analogRead(A0);
      digitalWrite(LUM_POWER_PIN,LOW);

      readDHT();
      String hi01 = (String)temperature;
      String hi02 = (String)humidity;
      String hi03 = (String)luminosity;
      hi01.toCharArray(valueStr01, 5);
      hi02.toCharArray(valueStr02, 5);
      hi03.toCharArray(valueStr03, 4);

    uint16_t packetIdPub1 = mqttClient.publish("balcony/sensor/temperature", 0, true, valueStr01);
    uint16_t packetIdPub2 = mqttClient.publish("balcony/sensor/humidity", 0, true, valueStr02);
    uint16_t packetIdPub3 = mqttClient.publish("balcony/sensor/luminosity", 0, true, valueStr03);
    

}
}

void readDHT(){
  // READ DATA
    Serial.print("DHT, \t");
//    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is active low on the ESP-01)
//    delay(50);                      // Wait for a second
//    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    int chk;
    uint32_t start = micros();
    switch (DHT_MODEL)
      {
      case 11:
        chk = DHT.read11(DHT_PIN);
        break;
      case 22:
        chk = DHT.read22(DHT_PIN);
        break;
      }      
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
    Serial.print(humidity, 1);
    Serial.print(",\t");
    temperature=DHT.temperature;
    Serial.print(temperature, 1);
    Serial.print(",\t");
    Serial.print(stop - start);
    Serial.println();

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

void blinker() {
  int state = digitalRead(LED_BUILTIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_BUILTIN, !state);     // set pin to the opposite state
}
