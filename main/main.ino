#include "config.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>      // BME280 by Adafruit 
#include <WiFi.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#define PIN_ON 3
#define SLEEP_SEC 180
#define SDA 19
#define SCL 18
#define BME280_ADDRESS (0x77)     // Default on LaskaKit module
#define ADC_PIN 0                 // ADC pin on LaskaKit Meteo mini
#define VOLTAGE_RATIO 1.7693877551

Adafruit_BME280 bme;

WiFiClient client;

float temperature;
float pressure;
float humidity;
float bat_voltage;

void go_to_sleep(){
  delay(1);
  // ESP Deep Sleep 
  digitalWrite(PIN_ON, LOW);   // Turn off the uSUP power
  Serial.println("ESP in sleep mode");
  esp_sleep_enable_timer_wakeup(SLEEP_SEC * 1000000);
  esp_deep_sleep_start();
}

void readBat(){
  bat_voltage = analogReadMilliVolts(ADC_PIN) * VOLTAGE_RATIO / 1000;
  Serial.println("Battery voltage " + String(bat_voltage) + "V");
}

void readBME(){
  temperature = bme.readTemperature();
  humidity    = bme.readHumidity();
  pressure    = bme.readPressure() / 100.0F;  
  
  Serial.print("Temp: "); Serial.print(temperature); Serial.println("°C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println("% RH");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println("hPa");
}

void WiFiConnection(){
  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void postData(){
  HTTPClient http;
  String tempString = "{\"state\": \"" + String(temperature) + "\", \"attributes\": {\"unit_of_measurement\": \"°C\", \"friendly_name\": \"Venkovní teplota\"}}";
  String humidityString = "{\"state\": \"" + String(humidity) + "\", \"attributes\": {\"unit_of_measurement\": \"%\", \"friendly_name\": \"Venkovní vlhkost\"}}";
  String voltageString = "{\"state\": \"" + String(bat_voltage) + "\", \"attributes\": {\"unit_of_measurement\": \"V\", \"friendly_name\": \"Venkovní napětí baterie\"}}";
  int httpCode;

  if(WiFi.status()== WL_CONNECTED) {
    http.begin(HA_VOLTAGE_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", HA_TOKEN);
    httpCode = http.POST(voltageString);
    Serial.println(voltageString);
    Serial.print("HTTP return code: ");
    Serial.println(httpCode);

    http.begin(HA_TEMP_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", HA_TOKEN);
    httpCode = http.POST(tempString);
    Serial.println(tempString);
    Serial.print("HTTP return code: ");
    Serial.println(httpCode);

    http.begin(HA_HUMIDITY_URL);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", HA_TOKEN);
    httpCode = http.POST(humidityString);
    Serial.println(humidityString);
    Serial.print("HTTP return code: ");
    Serial.println(httpCode);
   
  } else {
    Serial.println("Wi-Fi disconnected");
  }
}

void setup() {
  WiFi.mode( WIFI_OFF );
  delay( 1 );
  
  Serial.begin(115200);
  while(!Serial) {} // Wait
  Serial.println("Serial output initialized");

  pinMode(PIN_ON, OUTPUT);      // Set EN pin for uSUP stabilisator as output
  digitalWrite(PIN_ON, HIGH);   // Turn on the uSUP power

  // initilizace BME280 | BME280 Initialization
  Wire.begin(SDA,SCL);
  if (! bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Serial.println("-- Weather Station Scenario --");
  Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
  Serial.println("filter off");
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );
  delay(10);

  readBME();
  readBat();     

  WiFiConnection();
  postData();

  WiFi.disconnect(true);

  go_to_sleep();
}

void loop() {
  // no loop - the setup() ends with deep sleep
}
