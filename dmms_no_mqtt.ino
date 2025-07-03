#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// ==== Pins ====
#define DHTPIN 4
#define DHTTYPE DHT21
#define LDR_PIN 32
#define HUMIDIFIER_PIN 23
#define FAN_PIN 25
#define WIFI_STATUS_LED 27

// ==== Objects ====
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== Globals ====
float temperature = NAN;
float humidity = NAN;
float lux = NAN;
unsigned long lastDHTRead = 0;
const unsigned long DHT_INTERVAL = 2000;

#define WIFI_RECOVER_TIME_MS 30000
#define WIFI_CHECK_INTERVAL_MS 10000

// ==== FreeRTOS Task: Keep WiFi Alive ====
void keepWiFiAlive(void *parameter) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(WIFI_STATUS_LED, HIGH);
      Serial.println("[WiFi] Still connected");
      vTaskDelay(WIFI_CHECK_INTERVAL_MS / portTICK_PERIOD_MS);
      continue;
    }

    digitalWrite(WIFI_STATUS_LED, LOW);
    Serial.println("[WiFi] Disconnected. Reconnecting...");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Reconnecting...");
    lcd.setCursor(0, 1);
    lcd.print("Wait WiFi...");

    WiFi.begin();
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
      delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[WiFi] Reconnected!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi Restored");
      lcd.setCursor(0, 1);
      lcd.print(WiFi.localIP());
      delay(3000);
      lcd.clear();
    } else {
      Serial.println("[WiFi] Reconnect Failed");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WiFi Reconnect");
      lcd.setCursor(0, 1);
      lcd.print("FAILED");
      delay(3000);
    }

    vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  // I/O
  pinMode(HUMIDIFIER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(WIFI_STATUS_LED, OUTPUT);

  digitalWrite(HUMIDIFIER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(WIFI_STATUS_LED, LOW);

  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Mushroom System");
  delay(2000);
  lcd.clear();

  dht.begin();

  // WiFiManager: only for first connection
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  lcd.setCursor(0, 0);
  lcd.print("Connect WiFi...");
  bool res = wm.autoConnect("AutoConnectAP", "password");

  if (!res) {
    lcd.setCursor(0, 1);
    lcd.print("Failed. Reboot?");
    Serial.println("[WiFi] Setup failed");
    delay(3000);
  } else {
    digitalWrite(WIFI_STATUS_LED, HIGH);
    Serial.println("[WiFi] Connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(3000);
    lcd.clear();
  }

  // Start FreeRTOS WiFi monitoring task
  xTaskCreatePinnedToCore(
    keepWiFiAlive,
    "keepWiFiAlive",
    5000,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  // DHT reading
  if (millis() - lastDHTRead >= DHT_INTERVAL) {
    lastDHTRead = millis();
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      temperature = t;
      humidity = h;
    }
  }

  // LDR reading
  int ldrRaw = analogRead(LDR_PIN);
  float voltage = ldrRaw * (3.3 / 4095.0);
  float resistance = (3.3 - voltage) * 10000.0 / voltage;
  lux = 500 / (resistance / 1000.0);

  // Humidity Control
  if (!isnan(humidity) && humidity < 80) {
    digitalWrite(HUMIDIFIER_PIN, HIGH);
    digitalWrite(FAN_PIN, HIGH);
  } else {
    digitalWrite(HUMIDIFIER_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
  }

  // Serial Output
  Serial.print("Temp: "); Serial.print(temperature);
  Serial.print(" C, Hum: "); Serial.print(humidity);
  Serial.print(" %, Lux: "); Serial.println(lux);

  // LCD Output
  lcd.setCursor(0, 0);
  lcd.print("T:");
  if (!isnan(temperature)) lcd.print(temperature, 1); else lcd.print("--.-");
  lcd.print("C H:");
  if (!isnan(humidity)) lcd.print(humidity, 0); else lcd.print("--");
  lcd.print("% ");

  lcd.setCursor(0, 1);
  lcd.print("Lux:");
  lcd.print(lux, 0);
  lcd.print("       ");

  delay(500);
}

