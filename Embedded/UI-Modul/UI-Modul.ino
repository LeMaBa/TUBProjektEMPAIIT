#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h >
#include <ArduinoJson.h>

//Software Info
const char* SWVersion = "1.1";
const char* Date= "20210221";
const char* ConfigVersion = "UserInterfaceREV.1.0";

//Hardware Info
const char* deviceName = "ESP32-UIModul";
const char* deviceID = "ESP32-UIModul";
const char* deviceConfig = "UserinterfaceREV.1.0";

//WIFI INFO
const char* ssid[2] = {"FRITZ!Box 6490 Cable","TP-LINK_2438"};
const char* pass[2] = {"53146456397943290872","53444622"};
int wifiID = 1;


//MQTT INFO
const char* broker [2] = {"192.168.1.100","broker.emqx.io"};
int brokerID = 0;
const char* MqttTopic[3] = {"/productiv/userInput/physical",
                            "/productiv/sensorData",
                            "/productiv/userInput/digital"};             

// Global Var
int ButtonState = 15;
int ButtonSilent = 2;
const char* message;

long lastPress1 = 0;
long lastPress2 = 0;
long currentTime;
long intervalMsg = 500;

//VARMQTT
bool SystemState = 0;
bool SystemMode = 0;
float Temperatur = 0;
float CO2 = 0; 
float HUM = 0;
const char* SystemStateChar;
const char* SystemModeChar;


//init WIFI Client
WiFiClient espClient; 

//init MQTT Client
PubSubClient client(espClient);

//Setup OLED Display (32 or 64 Pixel)
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R2, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);



void setup() {
  //Setup Serial Communication
  Serial.begin(115200);
  Serial.println("Start Setup");

  //Setup Screen
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  
  // Setup Wifi Connection
  SetupWiFi();

  // Setup MQTT Server
  client.setServer(broker[brokerID], 1883);
  client.setCallback(callback);
  SetupMQTT();
  client.subscribe(MqttTopic[1]);
  client.subscribe(MqttTopic[2]);

//Setup input Devices
    // Buttonm
  pinMode(ButtonState, INPUT);
  pinMode(ButtonSilent, INPUT);

}


void loop() {

currentTime = millis(); // Set Systen Time  
//Check WiFi-Status
if (WiFi.status() != WL_CONNECTED){
  SetupWiFi();
  }

//Check MQTT Connection
if (!client.connected()){
  SetupMQTT();
  }
   
 client.loop(); //Run Clientloop for MQTT
 ButtonLogicSystemMode(); //Check Button action
 ButtonLogicSystemState();  //Check Button action
 printScreen(); //Print Screen (opt. if something has changed)

}


void ButtonLogicSystemState(){
//Button SystemState On/Off
  if(digitalRead(ButtonState) && (currentTime - lastPress1 > intervalMsg))
  {
    if(SystemState)
    {
      SystemState = false;
      SendState();
      lastPress1 = currentTime; 
    }else{
      SystemState = true;
      SendState();
      lastPress1 = currentTime; 
    }
  }
}

void ButtonLogicSystemMode(){

//Button SystemMode Silent/Normal
  if(digitalRead(ButtonSilent) && (currentTime - lastPress2 > intervalMsg))
  {
     if(SystemMode)
     {
      SystemMode = false;
      SendState();
      lastPress2 = currentTime; 
    }else{
      SystemMode = true;
      SendState();
      lastPress2 = currentTime; 
    }
  } 
}


//Send State and Mode via MQTT
void SendState(){
    
    StaticJsonDocument<96> doc;

    doc["sender_id"] = deviceName;
    doc["state"] = SystemState ;
    doc["silent"] = SystemMode ;
    
    char JSONmessageBuffer[100];

    String output;
    serializeJson(doc, output);
    
    message = output.c_str();
   
    if(client.publish(MqttTopic[0],message))
    {
      Serial.println("Done");
    }else{
     Serial.println("Message is not send");
    }
 }


  


//Setup WiFi Connection
void SetupWiFi(){
    delay(100);
    Serial.print("\nConnecting to ");
    Serial.print(ssid[wifiID]);

    WiFi.begin(ssid[wifiID], pass[wifiID]);

    while(WiFi.status() != WL_CONNECTED) 
    {
      delay(100);
      Serial.print("-");
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.drawStr(0,10,"Connecting to Wifi");
      u8g2.sendBuffer();          // transfer internal memory to the display
    }
    
   Serial.print("\nConnected to ");
   u8g2.clearBuffer();          // clear the internal memory
   u8g2.drawStr(0,10,"Wifi connected");
   u8g2.sendBuffer();          // transfer internal memory to the display
   Serial.println(ssid[wifiID]);
}

//Setup MQTT Connection
void SetupMQTT (){
  
    Serial.print("\nConnecting to ");
    Serial.println(broker[brokerID]);
    
    u8g2.clearBuffer();          // clear the internal memory, has issus with 128x64 Display
    u8g2.drawStr(0,20,"Connecting to MQTT");
    u8g2.sendBuffer();          // transfer internal memory to the display
          
    if(client.connect(deviceName))
     {
      Serial.print("\nConnected to");
      Serial.print(broker[brokerID]);
      u8g2.clearBuffer();          // clear the internal memory
      u8g2.drawStr(0,20,"Broker connected");
      u8g2.sendBuffer();          // transfer internal memory to the display
    } 
}


//Receive MQTT Message and change State and Mode
void callback(char* topic, byte* payload, unsigned int length) {

  StaticJsonDocument<96> dock;
  deserializeJson(dock, payload, length);

  if (strcmp(topic,MqttTopic[1])==0) {
    Temperatur = dock["temperature"];
    CO2 = dock["CO2_equiv"];
    HUM = dock["humidity"]; 
  }
  
  if (strcmp(topic,MqttTopic[2])==0) {
    SystemState = dock["state"];
    SystemMode = dock["silent"]; 
  }
  
}


//Print Temperature, Humidity, CO2-Equivalent, System State and System Mode on Screen
void printScreen(){
   
    char tempString[8];
    dtostrf(Temperatur,1,2,tempString);
    char humString[8];
    dtostrf(HUM,1,2,humString);
    char CO2String[8];
    dtostrf(CO2,1,2,CO2String);

    if(SystemState){
      SystemStateChar = "An";
    }else{
      SystemStateChar = "Aus";
    }
    if(SystemMode){
      SystemModeChar = "An";
    }else{
      SystemModeChar = "Aus";
    }
    
  
  u8g2.clearBuffer();          // clear the internal memory, has issus with 128x64 Display
  //Zeile 1
  u8g2.drawStr(1,11,"Temperatur:");
  u8g2.drawStr(71,11,tempString);  // write something to the internal memory
  u8g2.drawStr(103,11,"C");
  //Zeile 2
  u8g2.drawStr(1,22,"Luft%:");
  u8g2.drawStr(71,22,humString);  // write something to the internal memory
  u8g2.drawStr(103,22,"%");
  //Zeile 3
  u8g2.drawStr(1,33,"CO2:");
  u8g2.drawStr(71,33,CO2String);  // write something to the internal memory
  u8g2.drawStr(105,33,"ppm");
    //Zeile 4
  u8g2.drawStr(1,44,"Status:");
  u8g2.drawStr(71,44,SystemStateChar);  // write something to the internal memory´
  //Zeile 5
  u8g2.drawStr(1,55,"Mode:");
  u8g2.drawStr(71,55,SystemModeChar);  // write something to the internal memory
  u8g2.sendBuffer(); 
    
}
