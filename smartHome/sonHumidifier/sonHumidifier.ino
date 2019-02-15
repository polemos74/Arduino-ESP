#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

unsigned long previousMillis;

#include "wifi_credentials.h"

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883

// SONOFF pin numbers
#define STATUS_LED_PIN 13
#define RELAY_PIN 12
#define INDICATOR_LED_PIN 14
#define PUSH_BUTTON  0            // Onboard push button

volatile bool buttonFlag = false ;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

char clientName[] = {"sonHumidifier"};

void connectToWifi() {
    // Connect to WiFi access point.
  ////Serial.println(); ////Serial.println();
  ////Serial.print("Connecting to ");
  ////Serial.println(WLAN_SSID);

  WiFi.hostname(clientName);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(STATUS_LED_PIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
                                    // but actually the LED is on; this is because 
                                    // it is active low on the ESP-01)
    yield();
    delay(50);                      // Wait for a second
    digitalWrite(STATUS_LED_PIN, HIGH);  // Turn the LED off by making the voltage HIGH
    for (int a = 1 ; a < 10 ; a++){
    yield();
    delay(50);
    }

  }
  ////Serial.println();
  digitalWrite(STATUS_LED_PIN, LOW);
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
  uint16_t packetIdSub = mqttClient.subscribe("son/humidifier/command", 1);
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
if (String(topic) == "son/humidifier/command") {
    if (payload[1] == 'n')  {
    digitalWrite(RELAY_PIN,HIGH);
    digitalWrite(STATUS_LED_PIN,HIGH);
    digitalWrite(INDICATOR_LED_PIN,LOW);    
mqttClient.publish("son/humidifier/feedback", 1, true, "true");
  //Serial.println("Publishing at QoS 2");      
      } 
    if (payload[1] == 'f'){
    digitalWrite(RELAY_PIN,LOW);
    digitalWrite(STATUS_LED_PIN,LOW);
    digitalWrite(INDICATOR_LED_PIN,HIGH);    
mqttClient.publish("son/humidifier/feedback", 1, true, "false");
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
  pinMode(STATUS_LED_PIN,OUTPUT);
  digitalWrite(STATUS_LED_PIN,HIGH);
  pinMode(INDICATOR_LED_PIN,OUTPUT);
  digitalWrite(INDICATOR_LED_PIN,HIGH);
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
  mqttClient.setKeepAlive(5).setCleanSession(false).setWill("son/humidifier/lastwill", 2, true, "disconnected");

  connectToWifi();
  
  for (int a = 1 ; a < 10 ; a++){
    yield();
    delay(50);
    }

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
    ////Serial.println("Button pressed!");
    bool pinStatus = digitalRead(RELAY_PIN);
    ////Serial.println(pinStatus);
    if (pinStatus == true){
      uint16_t packetIdPub1 = mqttClient.publish("son/humidifier/relay", 1, true, "0");
      }
    else {
      uint16_t packetIdPub1 = mqttClient.publish("son/humidifier/relay", 1, true, "1");
      }
    pinStatus = !pinStatus;
    digitalWrite(RELAY_PIN, pinStatus);
    //digitalWrite(ONBOARD_LED, !pinStatus);
    buttonFlag = false;
    }
}
void buttonPressed(){
  buttonFlag = true;
}

