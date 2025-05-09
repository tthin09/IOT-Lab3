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

// Firmware version
constexpr char FIRMWARE_VERSION[] = "v1.1";

// Config
constexpr uint16_t telemetrySendInterval = 5000U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Wifi config
constexpr char WIFI_SSID[] = "Tri Thinh 1";
constexpr char WIFI_PASSWORD[] = "19001234";

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
void wifiTask(void *);
void connectThingsboardTask(void *);
void dhtTask(void *);

// Helper function
void scanWifi();
void connectWifi();
void connectThingsboard();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println("Current firmware version: " + String(FIRMWARE_VERSION));
  pinMode(DHT_PIN, OUTPUT);

  connectWifi();
  connectThingsboard();
  dht.begin();

  xTaskCreate(dhtTask, "DHT task", 4096, NULL, 1, NULL);
  xTaskCreate(wifiTask, "Wifi task", 8092, NULL, 2, NULL);
  xTaskCreate(connectThingsboardTask, "Thingsboard task", 8092, NULL, 2, NULL);
}

void scanWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("Scan start");

  // WiFi.scanNetworks will return the number of networks found.
  int n = WiFi.scanNetworks();
  Serial.println("Scan done");
  if (n == 0)
  {
    Serial.println("no networks found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.printf("%2d", i + 1);
      Serial.print(" | ");
      Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
      Serial.print(" | ");
      Serial.printf("%4d", WiFi.RSSI(i));
      Serial.print(" | ");
      Serial.printf("%2d", WiFi.channel(i));
      Serial.print(" | ");
      switch (WiFi.encryptionType(i))
      {
      case WIFI_AUTH_OPEN:
        Serial.print("open");
        break;
      case WIFI_AUTH_WEP:
        Serial.print("WEP");
        break;
      case WIFI_AUTH_WPA_PSK:
        Serial.print("WPA");
        break;
      case WIFI_AUTH_WPA2_PSK:
        Serial.print("WPA2");
        break;
      case WIFI_AUTH_WPA_WPA2_PSK:
        Serial.print("WPA+WPA2");
        break;
      case WIFI_AUTH_WPA2_ENTERPRISE:
        Serial.print("WPA2-EAP");
        break;
      case WIFI_AUTH_WPA3_PSK:
        Serial.print("WPA3");
        break;
      case WIFI_AUTH_WPA2_WPA3_PSK:
        Serial.print("WPA2+WPA3");
        break;
      case WIFI_AUTH_WAPI_PSK:
        Serial.print("WAPI");
        break;
      default:
        Serial.print("unknown");
      }
      Serial.println();
      delay(10);
    }
  }
  Serial.println("");

  // Delete the scan result to free memory for code below.
  WiFi.scanDelete();

  // Wait a bit before scanning again.
  delay(5000);
}

void dhtTask(void *pvParameters)
{
  while (1)
  {
    vTaskDelay(telemetrySendInterval);

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("Failed to read from DHT11");
      continue;
    }

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, ");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    tb.sendTelemetryData("temperature", temperature);
    tb.sendTelemetryData("humidity", humidity);
  }
}

void wifiTask(void *pvParameters)
{
  connectWifi();
  while (1)
  {
    vTaskDelay(telemetrySendInterval / 2);
    if (WiFi.status() != WL_CONNECTED)
    {
      connectWifi();
    }
  }
}

void connectWifi()
{
  Serial.print("Wifi SSID: " + String(WIFI_SSID) + ", Wifi password: " + String(WIFI_PASSWORD) + "\n");
  Serial.print("Connecting to wifi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print("Status: ");
    Serial.println(WiFi.status());
    attempts++;
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

void connectThingsboard()
{
  if (!tb.connected())
  {
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
    tb.sendAttributeData("firmwareVer", FIRMWARE_VERSION);
    Serial.println("Subscribe to ThingsBoard done");
  }
}

void loop()
{
  // empty block, freertos take care of all tasks
}