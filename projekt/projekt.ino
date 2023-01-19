
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define WIFI_SSID "Telekom-aU08qw"
#define WIFI_PASSWORD "mmnxdamx4g64"

// MQTT Broker
#define MQTT_HOST "test.mosquitto.org"

#define MQTT_PORT 1883

// Teplotné MQTT Topics
#define MQTT_PUB_TEMP "KRKEO/bme280/temperature"
#define MQTT_PUB_HUM "KRKEO/bme280/humidity"
#define MQTT_PUB_PRES "KRKEO/bme280/pressure"

// BME280 I2C
Adafruit_BME280 bme;
// premenne
float temp;
float hum;
float pres;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // uložý kedy bola teplota uložená naposledy
const long interval = 10000;        // Interval na publikovanie

void connectToWifi() {
  Serial.println("Pripájanie do Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Pripájanie do MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi Pripojené");
      Serial.println("IP adresa: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi stratilo spojenie");
      xTimerStop(mqttReconnectTimer, 0); // zaruči to aby sa nepripojilo znova mqtt počas pripájania znova na wifi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Pripojené do MQTT.");
  Serial.print("Linka: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Odpojené z MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


void onMqttPublish(uint16_t packetId) {
  Serial.print("Poslanie prijaté.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  // Zapnúť BME280 sensor 
  if (!bme.begin(0x76)) {
    Serial.println("Nenašiel sa BME280 sensor, skontrolujte prepojenie!");
    while (1);
  }
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);;
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  // čislol X počet sekund (interval = 10 sekund) 
  // pošle novú MQTT správu
  if (currentMillis - previousMillis >= interval) {
    // uloží posledne publikovanie
    previousMillis = currentMillis;
    // Nove BME280 sensor čitania
    temp = bme.readTemperature();
    //temp = 1.8*bme.readTemperature() + 32;
    hum = bme.readHumidity();
    pres = bme.readPressure()/100.0F;
    
    // Publikovanie MQTT správy na topic esp32/BME2800/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publikovanie na topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Správa: %.2f \n", temp);

    // Publikovanie MQTT správy na topic esp32/BME2800/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(hum).c_str());                            
    Serial.printf("Publikovanie na topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
    Serial.printf("Správa: %.2f \n", hum);

    // Publikovanie MQTT správy na topic esp32/BME2800/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES, 1, true, String(pres).c_str());                            
    Serial.printf("Publikovanie na topic %s at QoS 1, packetId: %i", MQTT_PUB_PRES, packetIdPub3);
    Serial.printf("Správa: %.3f \n", pres);
  }
}
