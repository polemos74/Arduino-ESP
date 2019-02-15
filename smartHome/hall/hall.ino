#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

unsigned long previousMillis;

#include "wifi_credentials.h"

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883

// SONOFF pin numbers
#define LED_BUILTIN D4
#define RELAY_PIN D1
#define PUSH_BUTTON  D5            // Onboard push button

volatile bool buttonFlag = false ;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

char valueStr01[5];
char valueStr02[5];
char clientName[] = {"hall"};

void connectToWifi() {
    // Connect to WiFi access point.
  //Serial.println(); //Serial.println();
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
  uint16_t packetIdSub = mqttClient.subscribe("hall/alarm/lamp/command", 1);
  //Serial.print("Subscribing at QoS 1, packetId: ");
  //Serial.println(packetIdSub);
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
if (String(topic) == "hall/alarm/lamp/command") {
    if (payload[1] == 'n')  {
    digitalWrite(RELAY_PIN,HIGH);
mqttClient.publish("hall/alarm/lamp/feedback", 1, true, "on");
  //Serial.println("Publishing at QoS 2");      
      } 
    if (payload[1] == 'f'){
   digitalWrite(RELAY_PIN,LOW);
mqttClient.publish("hall/alarm/lamp/feedback", 1, true, "off");
  //Serial.println("Publishing at QoS 2");      

     }
}
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
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
  pinMode (PUSH_BUTTON,INPUT_PULLUP);
  
  attachInterrupt(PUSH_BUTTON,buttonPressed,FALLING);


  //Serial.begin(115200);
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
  mqttClient.setKeepAlive(60).setCleanSession(false).setWill("hall/alarm/lastwill", 2, true, "disconnected");

  connectToWifi();

//  WiFi.printDiag(//Serial);

}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis > 1000){
  previousMillis = currentMillis;
checkButton();
}
}
void checkButton(){
  if (buttonFlag == true){
      uint16_t packetIdPub1 = mqttClient.publish("hall/alarm/reed", 1, true, "1");
      buttonFlag = false;
    for (int a = 1 ; a < 201 ; a++){
    yield();
    delay(50);
    }
      packetIdPub1 = mqttClient.publish("hall/alarm/reed", 1, true, "0");
    }
}
void buttonPressed(){
  buttonFlag = true;
}

