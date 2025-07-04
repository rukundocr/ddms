#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

// ==== LCD ====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Pins ====
#define DHTPIN 4
#define DHTTYPE DHT21
#define LDR_PIN 32
#define HUMIDIFIER_PIN 23
#define FAN_PIN 25
#define WIFI_STATUS_LED 27

DHT dht(DHTPIN, DHTTYPE);

// ==== WiFi + MQTT Config ====
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp1");
Adafruit_MQTT_Publish humFeed  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hum1");
Adafruit_MQTT_Publish luxFeed  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lux1");
Adafruit_MQTT_Subscribe actuatorFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/actuator1");

// ==== Shared Values ====
float temperature = NAN;
float humidity = NAN;
float lux = NAN;
bool actuatorCommand = false;

SemaphoreHandle_t sensorMutex;

// ==== FreeRTOS Task: Keep WiFi Alive ====
void keepWiFiAlive(void *parameter) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
      digitalWrite(WIFI_STATUS_LED, LOW);
    } else {
      digitalWrite(WIFI_STATUS_LED, HIGH);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

// ==== Task: Sensor Reading ====
void readSensorsTask(void *parameter) {
  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    int ldrRaw = analogRead(LDR_PIN);
    float voltage = ldrRaw * (3.3 / 4095.0);
    float resistance = (3.3 - voltage) * 10000.0 / voltage;
    float currentLux = 500 / (resistance / 1000.0);

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    if (!isnan(t)) temperature = t;
    if (!isnan(h)) humidity = h;
    lux = currentLux;
    xSemaphoreGive(sensorMutex);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// ==== Task: Actuator Control ====
void humidityControlTask(void *parameter) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    float hum = humidity;
    xSemaphoreGive(sensorMutex);

    if (actuatorCommand) {
      digitalWrite(HUMIDIFIER_PIN, HIGH);
      digitalWrite(FAN_PIN, HIGH);
    } else {
      if (hum < 80.0) {
        digitalWrite(HUMIDIFIER_PIN, HIGH);
        digitalWrite(FAN_PIN, HIGH);
      } else {
        digitalWrite(HUMIDIFIER_PIN, LOW);
        digitalWrite(FAN_PIN, LOW);
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ==== Task: MQTT ====
void mqttTask(void *parameter) {
  unsigned long lastPublish = 0;
  const unsigned long publishInterval = 8000;  // 8 seconds

  for (;;) {
    if (mqtt.connected()) {
      mqtt.processPackets(100);
    } else {
      mqtt.connect();
    }

    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) {
      if (subscription == &actuatorFeed) {
        actuatorCommand = atoi((char *)actuatorFeed.lastread);
      }
    }

    if (millis() - lastPublish > publishInterval) {
      xSemaphoreTake(sensorMutex, portMAX_DELAY);
      float t = temperature;
      float h = humidity;
      float l = lux;
      xSemaphoreGive(sensorMutex);

      // Format floats as strings for MQTT publish
      char tempStr[10];
      char humStr[10];
      char luxStr[10];

      dtostrf(t, 4, 1, tempStr);
      dtostrf(h, 4, 1, humStr);
      dtostrf(l, 4, 1, luxStr);

      tempFeed.publish(tempStr);
      humFeed.publish(humStr);
      luxFeed.publish(luxStr);

      lastPublish = millis();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  lcd.begin();
  lcd.backlight();

  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(WIFI_STATUS_LED, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.autoConnect("AutoConnectAP", "password");

  mqtt.subscribe(&actuatorFeed);

  sensorMutex = xSemaphoreCreateMutex();

  lcd.setCursor(0, 0);
  lcd.print("DMMS STARTING..");
  delay(2000);
  lcd.clear();

  xTaskCreatePinnedToCore(keepWiFiAlive, "WiFiMonitor", 4000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(readSensorsTask, "SensorRead", 4000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(humidityControlTask, "Actuation", 3000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask, "MQTT", 6000, NULL, 1, NULL, 1);
}

void loop() {
  xSemaphoreTake(sensorMutex, portMAX_DELAY);
  float t = temperature;
  float h = humidity;
  float l = lux;
  xSemaphoreGive(sensorMutex);

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t, 1);
  lcd.print("C H:");
  lcd.print(h, 0);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Lux:");
  lcd.print(l, 0);
  lcd.print("     ");

  delay(500);
}

