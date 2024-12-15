#define BLYNK_TEMPLATE_ID "TMPL3mAqq8Zkt"   // Replace with your Blynk Template ID
#define BLYNK_TEMPLATE_NAME "Iot Project"    // Replace with your Blynk Template Name
#define BLYNK_DEVICE_NAME "esp32"            // Replace with your Device Name

#include <WiFi.h>               // Wi-Fi library for ESP32
#include <BlynkSimpleEsp32.h>   // Blynk library for ESP32
#include "DHT.h"                // DHT library for temperature and humidity sensor

// Wi-Fi credentials
const char* ssid = "Galaxy A14 5G EF76";         // Replace with your Wi-Fi network name
const char* password = "siddrail131";           // Replace with your Wi-Fi password

// Blynk Auth Token
char auth[] = "LPbxpD2NBdSy4axpP1TWMxetO61hsKT5";          // Replace with your Blynk Auth Token

// Pin Definitions
const int relayPin = 4;       // GPIO4 connected to the relay module
const int waterPin = 2;       // GPIO2 corresponds to D2 on ESP32
const int dhtPin = 5;         // GPIO5 connected to the DHT22 data pin

// DHT sensor type
#define DHTTYPE DHT22

// Variables
int waterState;
DHT dht(dhtPin, DHTTYPE);     // Create a DHT object

// Virtual Pins for Blynk App
#define VIRTUAL_TEMP V1       // Virtual pin for temperature (Gauge)
#define VIRTUAL_HUMID V2      // Virtual pin for humidity (Gauge)
#define VIRTUAL_RELAY V3      // Virtual pin to control relay
#define VIRTUAL_WATER V4      // Virtual pin to display water sensor state (Gauge)

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(10);

  // Configure pins
  pinMode(relayPin, OUTPUT);       // Relay pin as output
  pinMode(waterPin, INPUT_PULLUP); // Use an internal pull-up resistor for the water sensor pin

  // Ensure the motor is OFF initially
  digitalWrite(relayPin, HIGH);    // Turn OFF relay (motor OFF)

  // Start DHT sensor
  dht.begin();

  // Connect to Wi-Fi and Blynk
  Serial.println("Connecting to Wi-Fi and Blynk...");
  Blynk.begin(auth, ssid, password);

  Serial.println("Connected to Wi-Fi and Blynk!");
}

void loop() {
  // Keep Blynk connected
  Blynk.run();

  // Read water sensor input
  waterState = digitalRead(waterPin);

  // Read temperature and humidity from DHT22
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Check if sensor readings are valid
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Send temperature and humidity data to Blynk gauges
  Blynk.virtualWrite(VIRTUAL_TEMP, temperature);    // Update temperature gauge
  Blynk.virtualWrite(VIRTUAL_HUMID, humidity);      // Update humidity gauge

  // Determine water sensor state and send to Blynk
  String waterStatus = (waterState == LOW) ? "Water Detected" : "No Water";
  
  // Send water status as a gauge (you can display this on a gauge as 0 or 100 or show a custom message)
  if (waterState == LOW) {
    Blynk.virtualWrite(VIRTUAL_WATER, 100); // Water detected
  } else {
    Blynk.virtualWrite(VIRTUAL_WATER, 0);   // No water detected
  }

  // Relay control logic based on water sensor
  if (waterState == LOW) { // Water detected
    Serial.println("Water Detected: Turning ON the Motor");
    digitalWrite(relayPin, LOW); // Turn ON the relay (motor ON)
    Blynk.virtualWrite(VIRTUAL_RELAY, 1); // Update relay state on Blynk
  } else { // No water detected
    Serial.println("No Water Detected: Turning OFF the Motor");
    digitalWrite(relayPin, HIGH); // Turn OFF the relay (motor OFF)
    Blynk.virtualWrite(VIRTUAL_RELAY, 0); // Update relay state on Blynk
  }

  // Debugging: Show the sensor values in the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Water sensor state: ");
  Serial.println(waterStatus);

  delay(2000); // Delay for stabilization
}