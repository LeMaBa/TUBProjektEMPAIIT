#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <PubSubClient.h >
#include <ArduinoJson.h>


//Software Info
const char* SWVersion = "1.1";
const char* Date= "20210221";
const char* ConfigVersion = "LefterModulREV.1.0";

//Hardware Info
const char* deviceName = "ESP32-LuefterModul-1";
const char* deviceID = "ESP32-LuefterModul-1";
const char* deviceConfig = "LuefterModulREV.1.0";

//WIFI INFO
const char* ssid[2] = {"",""};
const char* pass[2] = {"",""};
int wifiID = 1;


//MQTT INFO
const char* broker [2] = {"192.168.1.100","broker.emqx.io"};
int brokerID = 0;
const char* MqttTopic[2] = {"/productiv/machineData",
                            "/productiv/controllSystem"};             

//Interrupt Info
long lastMsg = 0;
long currentTime;
long intervalMsg = 2000;

//
const char* drehz;

int Luefter = 0;
int InterruptPin = 2;
volatile int counter = 0;

//Save Aktor settings
int lueftint = 0;

//init WIFI Client
WiFiClient espClient; 

//init MQTT Client
PubSubClient client(espClient);


//Setup Interrupt
//portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;



void callback(char* topic, byte* payload, unsigned int length) {
  //Lüfterleistung aus MQTT auslesen
  
  StaticJsonDocument<96> dock;
  deserializeJson(dock, payload, length);
  lueftint = dock["luefter_soll"];
  Serial.print(lueftint);
  analogWrite(Luefter, (lueftint)*10);

}


void setup() {
  //Setup Serial Communication
  Serial.begin(115200);
  Serial.println("Start Setup");
  
  // Setup Wifi Connection
  SetupWiFi();

  // Setup MQTT Server
  client.setServer(broker[brokerID], 1883);
  client.setCallback(callback);
  SetupMQTT();
  client.subscribe(MqttTopic[1]);

  //Setup output Devices
    //Setup Luefter
  pinMode(Luefter, OUTPUT);

  //Setup input Devices
    // Lüfter
  pinMode(InterruptPin, INPUT);

}


ICACHE_RAM_ATTR void isr(){
  counter++;
}


void loop() {
  
currentTime = millis(); // Set Systen Time
client.loop();//Run Clientloop for MQTT

//Check Wifi Connection
if (WiFi.status() != WL_CONNECTED){
  SetupWiFi();
  }
  
//Check MQTT Connection
if (!client.connected()){
  SetupMQTT();
  }

//Counter für Drehzahl
attachInterrupt(digitalPinToInterrupt(InterruptPin), isr, RISING);


//SEND MQTT Message
if(currentTime - lastMsg > intervalMsg){
  
    StaticJsonDocument<300> doc;

    doc["device"] = "ESP32";
    doc["drehzahl"] = (counter*15);
    doc["leistung"] = lueftint;

    counter = 0; //reset Counter
  
    char JSONmessageBuffer[100];

    String output;
    serializeJson(doc, output);

    drehz = output.c_str();
  

    if(client.publish(MqttTopic[0],drehz)){
      lastMsg = currentTime; 
      Serial.println("Done");
    }else{
     // Serial.println("Message is not send");
    }
    

    
  }
}



void SetupWiFi(){
    delay(100);
    Serial.print("\nConnecting to ");
    Serial.print(ssid[wifiID]);

    WiFi.begin(ssid[wifiID], pass[wifiID]);

    while(WiFi.status() != WL_CONNECTED) 
    {
      delay(100);
      Serial.print("-");
    }
    
   Serial.print("\nConnected to ");
   Serial.println(ssid[wifiID]);
}


void SetupMQTT (){
  
    Serial.print("\nConnecting to ");
    Serial.println(broker[brokerID]);
          
    if(client.connect(deviceName))
     {
      Serial.print("\nConnected to");
      Serial.print(broker[brokerID]);
     
    } 
}
