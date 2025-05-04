#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

#define SSID "HSU_Students"
#define PASSWORD "dhhs12cnvch"

// MQTT Broker details
#define MQTT_SERVER "broker.emqx.io"
const uint16_t MQTT_PORT = 1883;

#define MQTT_TOPIC_PUB_TEMP "ProjectKha/temp"
#define MQTT_TOPIC_SUB_TEMP "ProjectKha/temp"
#define MQTT_TOPIC_PUB_HUMID "ProjectKha/humid"
#define MQTT_TOPIC_SUB_HUMID "ProjectKha/humid"

WiFiClient espClient;
PubSubClient client(espClient);
StaticJsonDocument<256> doc;
ESP8266WebServer server(80);

// DHT sensor settings
#define DHTPIN D5  // Pin where the DHT sensor is connected
#define DHTTYPE DHT11  // Change this to DHT11 if you are using the DHT11 sensor
DHT dht(DHTPIN, DHTTYPE);

// Buzzer settings
#define BUZZER_PIN D6  // Pin connected to the buzzer
int temperatureThreshold = 30;  // Default temperature threshold
bool buzzerState = LOW;
bool userBuzzerControl = true;  // User control flag for the buzzer

// LCD settings
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns and 2 rows

void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.print("Connecting to existing Wifi network...");
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    // Initialize LCD
    lcd.init();  // Initialize the LCD
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");

    // Register URL handlers
    server.on("/Buzzer/on", HTTP_GET, handleBuzzerOn);
    server.on("/Buzzer/off", HTTP_GET, handleBuzzerOff);

    setup_wifi();
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(callback);

    dht.begin();
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer initially

    server.begin();  // Start the web server
    Serial.println("HTTP server started");
    lcd.setCursor(0, 1);
    lcd.print("HTTP server ready");
    reconnect();
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(SSID);

    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");

        if (client.connect("ProjectKha")) {
            Serial.println("connected");

            client.subscribe(MQTT_TOPIC_SUB_TEMP);
            client.subscribe(MQTT_TOPIC_SUB_HUMID);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(message);

    // Handle MQTT messages here if needed
}

float roundToDecimal(float value, int decimals) {
    float scale = pow(10, decimals);
    return round(value * scale) / scale;
}

void handleBuzzerOn() {
    userBuzzerControl = true;  // User wants to control the buzzer
    buzzerState = HIGH;  // Set flag to indicate that the buzzer is on
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on the buzzer
    server.send(200, "text/html", "Buzzer activated!");
}

void handleBuzzerOff() {
    userBuzzerControl = false;  // User wants to control the buzzer
    buzzerState = LOW;  // Clear flag to indicate that the buzzer is off
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
    server.send(200, "text/html", "Buzzer is off");
}

void loop() {
    server.handleClient();

    // MQTT communication
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Sensor readings and actions
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 2000) {  // Publish every 2 seconds
        lastPublish = millis();

        float h = dht.readHumidity();
        float t = dht.readTemperature();

        if (isnan(h) || isnan(t)) {
            Serial.println("Failed to read from DHT sensor!");
            lcd.setCursor(0, 0);
            lcd.print("Sensor error      ");
            return;
        }

        // Round temperature and humidity to 1 decimal place
        t = roundToDecimal(t, 1);
        h = roundToDecimal(h, 1);

        // Publish temperature and humidity
        doc["temperature"] = t;
        doc["humidity"] = h;
        char buffer[256];
        size_t n = serializeJson(doc, buffer);
        client.publish(MQTT_TOPIC_PUB_TEMP, buffer, n);
        client.publish(MQTT_TOPIC_PUB_HUMID, buffer, n);

        Serial.print("Temperature: ");
        Serial.print(t);
        Serial.print(" °C, Humidity: ");
        Serial.print(h);
        Serial.println(" %");

        // Display temperature and humidity on LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(t);
        lcd.print(" C");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(h);
        lcd.print(" %");

        // Check if temperature is above threshold to activate the buzzer
        if (t > temperatureThreshold && userBuzzerControl) {
            buzzerState = HIGH;  // Turn on the buzzer
        } else if (!userBuzzerControl) {
            buzzerState = LOW;  // Keep the buzzer off if user has turned it off
        }

        // If buzzer state is HIGH, turn on the buzzer
        if (buzzerState == HIGH) {
            digitalWrite(BUZZER_PIN, HIGH);
        } else {
            digitalWrite(BUZZER_PIN, LOW);
        }

        delay(100);  // Sleep 100ms to reduce CPU load
    }
}
