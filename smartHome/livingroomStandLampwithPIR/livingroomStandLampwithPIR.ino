#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>

unsigned long previousMillis;

#include "wifi_credentials.h"

#define MQTT_HOST IPAddress(192, 168, 0, 100)
#define MQTT_PORT 1883

#define ONBOARD_LED 13
#define RELAY_PIN 12              // Onboard Relay
#define INDICATOR_PIN 14
#define SENSOR01_OUTPUT_PIN    5  // PIR sensor
#define PUSH_BUTTON  0            // Onboard push button

volatile bool buttonFlag = false ;
volatile bool motionFlag = false ;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

char clientName[] = {"livingStandingLamp"};

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void connectToWifi() {
    // Connect to WiFi access point.
  //Serial.println(); //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(WLAN_SSID);

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
  //Serial.println();
  }
  //Serial.println("WiFi connected");
  //Serial.println("IP address: "); //Serial.println(WiFi.localIP());
  digitalWrite(ONBOARD_LED, LOW);
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
  uint16_t packetIdSub1 = mqttClient.subscribe("living/lamp/stand/command", 1);
  //Serial.print("Subscribing at QoS 1, packetId: ");
  //Serial.println(packetIdSub1);
  uint16_t packetIdSub2 = mqttClient.subscribe("living/pir/enable", 1);
  //Serial.print("Subscribing at QoS 1, packetId: ");
  //Serial.println(packetIdSub2);

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
  if (String(topic) == "living/lamp/stand/command") {
    if (payload[0] == '1')  {
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(LED_BUILTIN,HIGH);
      digitalWrite(INDICATOR_PIN,LOW);    
      mqttClient.publish("living/lamp/stand/feedback", 1, true, "true");
      //Serial.println("Publishing at QoS 2");      
      } 
    if (payload[0] == '0'){
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_BUILTIN,LOW);
      digitalWrite(INDICATOR_PIN,HIGH);    
      mqttClient.publish("living/lamp/stand/feedback", 1, true, "false");
      //Serial.println("Publishing at QoS 2");      
      }
}
/*if (String(topic) == "living/pir/enable") {
    if (payload[0] == '1')  {
      digitalWrite(SENSOR01_ENABLE_PIN, HIGH);
mqttClient.publish("living/lamp/stand/feedback", 2, true, "true");
  //Serial.println("Publishing at QoS 2");      
      } 
     if (payload[0] == '0'){
      digitalWrite(SENSOR01_ENABLE_PIN, LOW);
mqttClient.publish("living/lamp/stand/feedback", 2, true, "false");
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
  pinMode(ONBOARD_LED,OUTPUT);
  digitalWrite(ONBOARD_LED,HIGH);
  pinMode(INDICATOR_PIN,OUTPUT);
  digitalWrite(INDICATOR_PIN,HIGH);
  pinMode (RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
  pinMode (PUSH_BUTTON,INPUT_PULLUP);
  pinMode (SENSOR01_OUTPUT_PIN,INPUT);
  
  attachInterrupt(PUSH_BUTTON,buttonPressed,FALLING);
  attachInterrupt(SENSOR01_OUTPUT_PIN,motionDetected,RISING);

  //Serial.begin(115200);
  //Serial.println();
  //Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setKeepAlive(5).setCleanSession(false).setWill("living/lamp/stand/lastwill", 2, true, "disconnected");


  connectToWifi();
  //WiFi.printDiag(//Serial);

}

void loop() {
  checkButton();
  checkPIR();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > 1000){
  previousMillis = currentMillis;
  //Serial.print(".");
  // digitalWrite(ONBOARD_LED,LOW);
  //delay (25);
  // digitalWrite(ONBOARD_LED,HIGH);
  }
}

void checkButton(){
  if (buttonFlag == true){
  ////Serial.println("Button pressed!");
  bool pinStatus = digitalRead(RELAY_PIN);
  ////Serial.println(pinStatus);
  if (pinStatus == true){
    uint16_t packetIdPub1 = mqttClient.publish("living/lamp/stand/feedback", 1, true, "0");
    }
  else {
    uint16_t packetIdPub1 = mqttClient.publish("living/lamp/stand/feedback", 1, true, "1");
    }
  pinStatus = !pinStatus;
  digitalWrite(RELAY_PIN, pinStatus);
  digitalWrite(ONBOARD_LED, !pinStatus);
  buttonFlag = false;
  }
}

void checkPIR(){
  if (motionFlag == true){
    uint16_t packetIdPub1 = mqttClient.publish("living/pir/sense", 1, true, "1");
    //Serial.print("Motion detected. Publishing at QoS 0, packetId: ");
    //Serial.println(packetIdPub1);
    motionFlag = false;
    for (int a = 1 ; a < 60 ; a++){
    yield();
    delay(50);
    }
    packetIdPub1 = mqttClient.publish("living/pir/sense", 1, true, "0");    
    //packetIdPub1 = mqttClient.publish("living/pir/sense", 1, true, "0");
    }
}

void buttonPressed(){
  buttonFlag = true;
}

void motionDetected(){
  motionFlag = true;
}
  
