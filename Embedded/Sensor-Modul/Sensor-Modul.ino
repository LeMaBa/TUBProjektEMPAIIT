//Bibliotheken einbinden
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include "Adafruit_BME680.h"
#include <WiFi.h>
#include <PubSubClient.h >
#include <ArduinoJson.h>
#include <bsec.h>


//WIFI INFO
const char* ssid = "FRITZ!Box 6490 Cable";
const char* pass = "53146456397943290872";

//MQTT INFO
const char* broker [2] = {"192.168.0.39","broker.emqx.io"};
int brokerID = 0;

const char* MqttTopic[3] = "/test/hum/Outside",
                            "/test/temp/Outside",
                            "/test/Maschine/UserInput"};



const char* outTopicI = "/test/hum/Outside";
const char* outTopicT = "/test/temp/Outside";
const char* inTopicU = "/test/Maschine/UserInput";
const char* intopicS = "";

//Interrupt Info
long lastMsg = 0;
long currentTime;
long intervalMsg = 2000;

const char* temp;

float temperature = 0;
char tempString[8];

String output;

// Create an object of the class Bsec
Bsec iaqSensor;


//init WIFI Client
WiFiClient espClient; 

//init MQTT Client
PubSubClient client(espClient);




void setup() {
  //Setup Serial Communication
  Serial.begin(115200);
  Serial.println("Start Setup");

  Wire.begin();

//Initialise Sensor
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
 
  checkIaqSensorStatus();

//Load Sensor Data
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };
  
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();



  
  // Setup Wifi Connection
  SetupWiFi();

  // Setup MQTT Server
  client.setServer(broker [brokerID], 1883);
  SetupMQTT();
  client.subscribe(MqttTopic[2]);

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


//GET Data & SEND MQTT Message every 2 seconds
if(currentTime - lastMsg > intervalMsg){

//Get Temp, presure, humidity, gas resistance and CO2-equivalent
    if (iaqSensor.run()) {
    Serial.print(F("Temperature = "));
    Serial.print(String(iaqSensor.temperature));
    Serial.println(F(" *C"));
    temperature = iaqSensor.temperature;
    dtostrf(iaqSensor.temperature,1,2,tempString);

    Serial.print(F("Pressure = "));
    Serial.print(iaqSensor.pressure / 100.0);
    Serial.println(F(" hPa"));

    Serial.print(F("Humidity = "));
    Serial.print(String(iaqSensor.rawHumidity));
    Serial.println(F(" %"));
    char humString[8];
    dtostrf(iaqSensor.humidity,1,2,humString);

    Serial.print(F("Gas = "));
    Serial.print(iaqSensor.gasResistance / 1000.0);
    Serial.println(F(" KOhms"));


    Serial.print(F("CO2-Equivalent = "));
    Serial.print(String(iaqSensor.co2Equivalent));
    Serial.println(F(" ppm"));
  
  
    Serial.print("Sending message: ");  
  
    StaticJsonDocument<300> doc;

/Send data via MQTT
    doc["device"] = "ESP32";
    doc["sensorType"] = "Temperature";
    doc["CO2_equiv"] = String(iaqSensor.co2Equivalent);
    doc["temperature"] = String(iaqSensor.temperature);
    doc["humidity"] = String(iaqSensor.humidity);

  
    char JSONmessageBuffer[100];

    String output2;
    serializeJson(doc, output2);
    temp = output2.c_str();
  

    if(client.publish(MqttTopic[1],temp)){
      lastMsg = currentTime; 
      Serial.println("Done");
    }else{
      Serial.println("Message is not send");
    }
      }else {
      checkIaqSensorStatus();
      }
    }
}


//Check Sensor Status
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}



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


void SetupMQTT (){
  
    Serial.print("\nConnecting to ");
    Serial.println(broker [brokerID]);
          
    if(client.connect("ESP32-Outside"))
     {
      Serial.print("\nConnected to");
      Serial.print(broker [brokerID]);
    } 
}





void errLeds(void)
{
  Serial.println("error");
}
