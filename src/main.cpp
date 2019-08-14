#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP085.h>
extern "C" {
#include "user_interface.h"
}
#include <Adafruit_Sensor.h>
#include "DHT.h"

#define SSID "XXXXXXXXXXX"
#define PASSWD "XXXXX"

#define MQTT_SERVER "XX.XX.XX.XX"

#define REQUEST "request"
#define RESPONSE "response"
#define LOG "log"
#define WEATHER_STATION_ID "00000001"
#define MQTT_TOPIC_SENSOR_DATA REQUEST "/" WEATHER_STATION_ID "/sensorData"
#define CURRENT_VERSION_TOPIC REQUEST "/" WEATHER_STATION_ID "/version"
#define MQTT_TOPIC_FOTA_RESPONSE RESPONSE "/" WEATHER_STATION_ID "/fota"
#define DHTPIN 12    // Digital pin connected to the DHT sensor (D6)
#define DHTTYPE DHT22

const char version_s[11] = "1.0.0";
char buffer_c[100];

// DHT22
DHT dht(DHTPIN, DHTTYPE);

// BMP180
Adafruit_BMP085 bmp;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

bool setup_wifi(void);
bool connect_to_mqtt_server(void) ;
void callback(char* topic, byte* payload, unsigned int length);
int check_fota(void);
void start_fota(char* fwName);
void enter_in_sleep_mode(void);
void disable_wifi(void);
int push_data(const char* topic, const char* data);

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);     //Facultatif pour le debug
    
    delay(100);
    
    char macAddress[17];
    strcpy(macAddress, WiFi.macAddress().c_str());

    Serial.println("");
    Serial.println("+-----------------------------------------");
    Serial.printf( "| App version : %25s |\n", version_s);
    Serial.printf( "| DeviceId    : %25s |\n", WiFi.macAddress().c_str());
    Serial.printf( "| SSID        : %25s | \n", SSID);
    Serial.println("+-----------------------------------------");
        
    client.setServer(MQTT_SERVER, 1883);    //Configuration de la connexion au serveur MQTT
    client.setCallback(callback);  //La fonction de callback qui est executée à chaque réception de message     

    dht.begin();
    bmp.begin();
}

bool setup_wifi(void){
  delay(10);
  Serial.printf("Connecting to %s (1 min max) \n", SSID);
  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWD);

  unsigned long t = millis();
  bool delayExceeded = false;
  // Wait while wifi is not connected or delay is exceeded (1 min)  
  while (WiFi.status() != WL_CONNECTED && !delayExceeded) {
    delay(100);
    Serial.printf("\r  Waiting : %f sec", (millis() - t) / 1000.0);
    if (millis() - t > 1 * 60 * 1000) delayExceeded = true;
  }

  if (delayExceeded)
  {
    Serial.printf("\nFailed to connect to ssid %s \n", SSID);
    return false;
  }
  else
  {
    Serial.println("\nConnection established ");
    Serial.printf("=> IP Addresse : %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
}

// Reconnexion
bool connect_to_mqtt_server() {
  //Boucle jusqu'à obtenur une reconnexion
  if (setup_wifi())
  {
    Serial.println("Connecting to MQTT server (1 min max)");
    unsigned long t = millis();
    bool delayExceeded = false;
    while (!client.connected() && !delayExceeded) {
      delay(100);
      Serial.printf("\r  Waiting : %f sec", (millis() - t) / 1000.0);
      if (client.connect("WeatherStationFrancis"))
      {
        Serial.println("OK");
        client.subscribe(RESPONSE "/#");
      }      
      if (millis() - t > 1 * 60 * 1000) delayExceeded = true;
    }
    if (delayExceeded)
    {
      Serial.println("\nFailed to connect broker");
      return false;      
    }
    else
    {
      Serial.println("\nConnection established ");
      return true;
    }
    
  }
  else
  {
    return false;
  }
  
}

void loop() {
    // put your main code here, to run repeatedly:
    float dht_temp = dht.readTemperature();

    Serial.print("Temperature : ");
    Serial.println(dht_temp);

    float bmp_temp = bmp.readTemperature();

    Serial.print("Temperature: ");
    Serial.println(bmp_temp);

    uint32_t bmp_pressure = bmp.readPressure() / 100;
    
    Serial.print("Pression: ");
    Serial.println(bmp_pressure);

    float dht_hum = dht.readHumidity();
    Serial.print("Humidity : ");
    Serial.println(dht_hum);

    float bmp_alt = bmp.readAltitude() * (-1);
    Serial.print("Altitude : ");
    Serial.println(bmp_alt);
    Serial.println();
    
    const int capacity = JSON_OBJECT_SIZE(5);
    StaticJsonDocument<capacity> doc;   

    doc["Temp_DHT11"] = dht_temp;
    doc["Humid_DHT"] = dht_hum;
    doc["Temp_BMP180"] = bmp_temp;
    doc["Pressure_BMP180"] = bmp_pressure;
    doc["Altitud_BMP"] = bmp_alt;
    
    if (connect_to_mqtt_server())
    {
    
      client.loop();  

      char JSONmessageBuffer[128];
      serializeJson(doc, JSONmessageBuffer, sizeof(JSONmessageBuffer));
      push_data(MQTT_TOPIC_SENSOR_DATA, JSONmessageBuffer);

      // Send current verison of firmware
      push_data(CURRENT_VERSION_TOPIC, version_s);

      Serial.println("Wait 5 seconds, to let at server the time to send FOTA request");
      unsigned long t = millis();
      while (millis() - t <= 5 * 1000){
          delay(100); 
          client.loop();
          Serial.printf("\rWaiting : %f sec", (millis() -t) / 1000.0);
      }
      Serial.println("\nDisconnecting mosquitto client ...");
      client.disconnect();
    }
    enter_in_sleep_mode();
}

// Déclenche les actions à la réception d'un message
void callback(char* topic, byte* payload, unsigned int length) {
    char fw_name[256];
    strncpy(fw_name, (char*)payload, length); 
    Serial.println("Message recu =>  ");
    Serial.printf("  - topic: %s\n", topic);  
    Serial.printf("  - Payload: %s\n", (char*)fw_name);
    Serial.printf("Length = %d \n", length);
    
    if (strcmp(topic, MQTT_TOPIC_FOTA_RESPONSE) == 0)
    {
        start_fota((char*)payload);
    }
}

void start_fota(char* fwName){
    Serial.println("Staring FOTA ...");
    Serial.printf("Downloading %s\n", fwName);
    t_httpUpdate_return ret = ESPhttpUpdate.update(fwName);
    // si success, ESP will restart, else display error
    switch(ret)
    {
        case HTTP_UPDATE_FAILED:
            Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("HTTP_UPDATE_NO_UPDATES");
            break;
        default:
            Serial.printf("Unkown error code : %d\n", ret);
            break;
    }
}

void disable_wifi(void)
{
  // todo : Comment two below lines (to lower the power during LIGHT_SLEEP_T delay)
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  // Sleeping for 15 min
  WiFi.forceSleepBegin(15 * 60 * 1000);
  delay(100);
}

void enter_in_sleep_mode(void){
    Serial.println("Switch off WIFI");
    disable_wifi();
    Serial.println("Wait 15 min");
    unsigned long t = millis();
    while (millis() - t <= 15 * 60 * 1000){
        delay(100); 
        Serial.printf("\rWaiting : %f", (millis() -t) / 1000.0);
    }
    Serial.println("");
}


int push_data(const char* topic, const char* data)
{
  Serial.println("Try to publish :");
  Serial.printf("   -- Topic : %s \n", topic);
  Serial.printf("   -- Data  : %s \n", data);
  int res = 0;
  int retry = 0;
    do 
    {
        res = client.publish(topic, data);
        if (!res)
        {
        Serial.print("Failed to send Mqtt Data => Topic : ");
        Serial.print(topic);
        Serial.print(" , Data : ");
        Serial.println(data);
        
        // Wait 1 sec before to retry
        delay(1000);
        }
        else
        {
        Serial.println("Publish OK");  
        }
        retry += 1;
    }while(!res && retry < 10);
  return res;
}