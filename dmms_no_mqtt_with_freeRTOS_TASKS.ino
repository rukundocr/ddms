#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <WiFi.h>
#include <WiFiManager.h>

// ==== Pin Definitions ====
#define DHTPIN 4
#define DHTTYPE DHT21
#define LDR_PIN 32
#define HUMIDIFIER_PIN 23
#define FAN_PIN 25
#define WIFI_STATUS_LED 27

// ==== Objects ====
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Shared Sensor Values ====
float temperature = NAN;
float humidity = NAN;
float lux = NAN;
SemaphoreHandle_t sensorMutex;

// ==== WiFi Monitor Task ====
void keepWiFiAlive(void *parameter) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(WIFI_STATUS_LED, HIGH);
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }

    digitalWrite(WIFI_STATUS_LED, LOW);
    WiFi.begin();
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
      delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(WIFI_STATUS_LED, HIGH);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi Restored");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      delay(3000);
      lcd.clear();
    }

    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}

// ==== Task: Read DHT21 + LDR ====
void readSensorsTask(void *parameter) {
  for (;;) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    int ldrRaw = analogRead(LDR_PIN);
    float voltage = ldrRaw * (3.3 / 4095.0);
    float resistance = (3.3 - voltage) * 10000.0 / voltage;
    float currentLux = 500 / (resistance / 1000.0);

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    if (!isnan(t) && !isnan(h)) {
      temperature = t;
      humidity = h;
    }
    lux = currentLux;
    xSemaphoreGive(sensorMutex);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// ==== Task: Control Humidifier and Fan ====
void humidityControlTask(void *parameter) {
  for (;;) {
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    float currentHum = humidity;
    xSemaphoreGive(sensorMutex);

    if (!isnan(currentHum) && currentHum < 80.0) {
      digitalWrite(HUMIDIFIER_PIN, HIGH);
      digitalWrite(FAN_PIN, HIGH);
    } else {
      digitalWrite(HUMIDIFIER_PIN, LOW);
      digitalWrite(FAN_PIN, LOW);
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  lcd.begin();
  lcd.backlight();

  // Init outputs
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(WIFI_STATUS_LED, OUTPUT);

  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(WIFI_STATUS_LED, LOW);

  // Welcome message
  lcd.setCursor(0, 0);
  lcd.print("Mushroom System");
  delay(2000);
  lcd.clear();

  // WiFi setup
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  bool res = wm.autoConnect("AutoConnectAP", "password");

  if (!res) {
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    delay(3000);
  } else {
    digitalWrite(WIFI_STATUS_LED, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(3000);
    lcd.clear();
  }

  // Create mutex
  sensorMutex = xSemaphoreCreateMutex();

  // FreeRTOS Tasks
  xTaskCreatePinnedToCore(keepWiFiAlive, "WiFiMonitor", 5000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(readSensorsTask, "SensorRead", 4000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(humidityControlTask, "HumidityControl", 3000, NULL, 1, NULL, 1);
}

void loop() {
  // Show sensor values on LCD
  xSemaphoreTake(sensorMutex, portMAX_DELAY);
  float t = temperature;
  float h = humidity;
  float l = lux;
  xSemaphoreGive(sensorMutex);

  lcd.setCursor(0, 0);
  lcd.print("T:");
  if (!isnan(t)) lcd.print(t, 1); else lcd.print("--.-");
  lcd.print("C H:");
  if (!isnan(h)) lcd.print(h, 0); else lcd.print("--");
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Lux:");
  lcd.print(l, 0);
  lcd.print("      ");

  delay(500);
}

