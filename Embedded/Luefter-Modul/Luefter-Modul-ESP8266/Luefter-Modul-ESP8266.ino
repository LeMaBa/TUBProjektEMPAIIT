#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
//#include <WiFi.h>
#include <PubSubClient.h >
#include <ArduinoJson.h>

//WIFI INFO
const char* ssid = "FRITZ!Box 6490 Cable";
const char* pass = "53146456397943290872";

//MQTT INFO
const char* broker = "broker.emqx.io";
//const char* broker = "192.168.0.39";
const char* outTopicI = "/test/temp/Outside";
const char* outTopicT = "/test/Maschine/Status";
const char* inTopicU = "/test/Maschine/UserInput";
const char* intopicS = "";

//Interrupt Info
long lastMsg = 0;
long currentTime;
long intervalMsg = 2000;

//
const char* drehz;

//Set Pins
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
  client.setServer(broker, 1883);
  client.setCallback(callback);
  SetupMQTT();
  client.subscribe(inTopicU);

  //Setup output Devices
    //Setup Luefter
  pinMode(Luefter, OUTPUT);

  //Setup input Devices
    // Lüfter
  pinMode(InterruptPin, INPUT);

}


//Increase counter by 1
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




//SEND MQTT Message every 2 seconds
if(currentTime - lastMsg > intervalMsg){
  
    StaticJsonDocument<300> doc;

//Send RPM and Power % via MQTT
    doc["device"] = "ESP8266";
    doc["drehzahl"] = (counter*15);
    doc["leistung"] = lueftint;

    counter = 0; //reset Counter
 
    JsonArray values = doc.createNestedArray("values");
  
    char JSONmessageBuffer[100];

    String output;
    serializeJson(doc, output);

    drehz = output.c_str();
  

    if(client.publish(outTopicT,drehz)){
      lastMsg = currentTime; 
      Serial.println("Done");
    }else{
     // Serial.println("Message is not send");
    }
    
  }
}


//Set up WiFi Connection
void SetupWiFi(){
    delay(100);
    Serial.print("\nConnecting to ");
    Serial.print(ssid);

    WiFi.begin(ssid, pass);

    while(WiFi.status() != WL_CONNECTED) 
    {
      delay(100);
      Serial.print("-");
    }
    
   Serial.print("\nConnected to ");
   Serial.println(ssid);
}


//Setup MQTT Connection
void SetupMQTT (){
  
    Serial.print("\nConnecting to ");
    Serial.println(broker);
          
    if(client.connect("ESP32-Outside"))
     {
      Serial.print("\nConnected to");
      Serial.print(broker);
     
    } 
}
