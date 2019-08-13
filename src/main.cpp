#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>

#define SSID "XXXXXXXXXXX"
#define PASSWD "XXXXX"

#define MQTT_SERVER "XX.XX.XX.XX"

#define REQUEST "request"
#define RESPONSE "response"
#define LOG "log"
#define WEATHER_STATION_ID "00000001"
#define MQTT_TOPIC_SENSOR_DATA RESPONSE "/" WEATHER_STATION_ID "/sensorData"
#define CURRENT_VERSION_TOPIC REQUEST "/" WEATHER_STATION_ID "/version"
#define MQTT_TOPIC_FOTA_RESPONSE RESPONSE "/" WEATHER_STATION_ID "/fota"


const char version_s[11] = "1.0.0.0";
char buffer_c[100];


WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup_wifi(void);
void reconnect(void) ;
void callback(char* topic, byte* payload, unsigned int length);
int check_fota(void);
void start_fota(char* fwName);
void enter_in_sleep_mode(void);
void disable_wifi(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     //Facultatif pour le debug
  delay(100);
  Serial.println("");
  Serial.println("+-----------------------------------------");
  Serial.printf( "| App version : %25s |\n", version_s);
  char macAddress[17];
  strcpy(macAddress, WiFi.macAddress().c_str());
  Serial.printf( "| DeviceId    : %25s |\n", WiFi.macAddress().c_str());
  Serial.printf( "| SSID        : %25s | \n", SSID);
  Serial.println("+-----------------------------------------");
  setup_wifi();           //On se connecte au réseau wifi  
  client.setServer(MQTT_SERVER, 1883);    //Configuration de la connexion au serveur MQTT
  client.setCallback(callback);  //La fonction de callback qui est executée à chaque réception de message     
}

void setup_wifi(){
  delay(10);
  Serial.printf("\nConnecting to %s ", SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnection established ");
  Serial.printf("=> IP Addresse : %s\n", WiFi.localIP().toString().c_str());
}

//Reconnexion
void reconnect() {
  //Boucle jusqu'à obtenur une reconnexion
  setup_wifi();
  while (!client.connected()) {
    Serial.print("Connexion au serveur MQTT... ");
    if (client.connect("WeatherStationFrancis"))
    {
      Serial.println("OK");
	  client.subscribe(RESPONSE "/#");
    } 
    else 
    {
      Serial.print("KO, erreur : ");
      Serial.print(client.state());
      Serial.println(" On attend 5 secondes avant de recommencer");
      delay(5000);
    }
  }
}

void loop() {
	// put your main code here, to run repeatedly:
	if (!client.connected()) {
		reconnect();
	}
	
	client.loop();	

	// Send current verison of firmware
	Serial.println("Publish current version");
	client.publish(CURRENT_VERSION_TOPIC, version_s);

	Serial.println("Wait 5 seconds, to let at server the time to send FOTA request");
	unsigned long t = millis();
	while (millis() - t <= 5 * 1000){
		delay(100);	
		client.loop();
		Serial.printf("\rWaiting : %f", (millis() -t) / 1000.0);
	}
	Serial.println("\nDisconnecting mosquitto client ...");
	client.disconnect();
	enter_in_sleep_mode();
}

// Déclenche les actions à la réception d'un message
void callback(char* topic, byte* payload, unsigned int length) {
	char fw_name[256];
	strncpy(fw_name, (char*)payload, length); 
	Serial.println("Message recu =>  ");
	Serial.printf("  - topic: %s\n", topic);  
	Serial.printf("  - Payload: %s\n", (char*)fw_name);
	Serial.printf("Length = %d", length);
	
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
	WiFi.mode(WIFI_OFF);
	WiFi.forceSleepBegin();
}

void enter_in_sleep_mode(void){
	Serial.println("Switch off WIFI");
	disable_wifi();
	Serial.println("Wait 10 min");
	unsigned long t = millis();
	while (millis() - t <= 1 * 60 * 1000){
		delay(100);	
		client.loop();
		Serial.printf("\rWaiting : %f", (millis() -t) / 1000.0);
	}
	Serial.println("");
}