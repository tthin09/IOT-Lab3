#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include "WiFi.h"

#define DHT_PIN 13
#define DHT_TYPE DHT11

DHT dht(DHT_PIN, DHT_TYPE);

// Config
constexpr uint16_t telemetrySendInterval = 5000U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Wifi config
constexpr char WIFI_SSID[] = "Redmi";
constexpr char WIFI_PASSWORD[] = "trithinh";

// Thingsboard setup
constexpr char TOKEN[] = "crr1n3ilqu162iug4e9h";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

// Objects
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// Tasks for RTOS
void wifiTask(void*);
void connectThingsboardTask(void*);
void dhtTask(void*);

// Helper function
void connectWifi();
void connectThingsboard();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(DHT_PIN, OUTPUT);

  connectWifi();
  connectThingsboard();
  dht.begin();

  xTaskCreate(dhtTask, "DHT task", 4096, NULL, 1, NULL);
  xTaskCreate(wifiTask, "Wifi task", 8092, NULL, 2, NULL);
  xTaskCreate(connectThingsboardTask, "Thingsboard task", 8092, NULL, 2, NULL);
}


void dhtTask(void *pvParameters)
{
  while (1)
  {
    vTaskDelay(telemetrySendInterval);

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT11");
      continue;
    }

    Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" °C, ");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    tb.sendTelemetryData("temperature", temperature);
    tb.sendTelemetryData("humidity", humidity);
  }
}

void wifiTask(void *pvParameters) {
  connectWifi();
  while (1) {
    vTaskDelay(telemetrySendInterval / 2);
    if (WiFi.status() != WL_CONNECTED) {
      connectWifi();
    }
  }
}

void loop() {
  // empty block, freertos take care of all tasks
}

void connectWifi() {
  Serial.print("Connecting to wifi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to wifi!");
}



void connectThingsboardTask(void *pvParameters)
{
  while (1)
  {
    connectThingsboard();
    vTaskDelay(5000);
  }
}

void connectThingsboard() {
  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.println(" with token ");
    const bool successful = tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
    if (!successful)
    {
      Serial.println("Failed to connect to ThingsBoard");
      return;
    }

    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    Serial.println("Subscribe to ThingsBoard done");
  }
}