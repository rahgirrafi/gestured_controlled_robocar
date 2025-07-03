#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Server configuration
const char* serverIP = "192.168.1.50";  // Python server's IP
const int serverPort = 8080;

// Car control pins
const int motorPin = 25;
const int steeringPin = 26;

// Reconnect settings
const unsigned long RECONNECT_INTERVAL = 5000;  // 5 seconds
unsigned long lastReconnectAttempt = 0;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);
  pinMode(steeringPin, OUTPUT);
  
  // Initial motor stop
  analogWrite(motorPin, 0);
  
  connectToWiFi();
  connectToServer();
}

void loop() {
  // Maintain server connection
  if (!client.connected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReconnectAttempt >= RECONNECT_INTERVAL) {
      lastReconnectAttempt = currentMillis;
      connectToServer();
    }
    return;
  }

  // Process incoming commands
  while (client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    Serial.println("Received: " + command);
    
    // Parse command (format: "SPEED,DIRECTION")
    int commaIndex = command.indexOf(',');
    if (commaIndex == -1) return;
    
    String speed = command.substring(0, commaIndex);
    String direction = command.substring(commaIndex + 1);
    
    // Process speed command
    if (speed == "25%") {
      analogWrite(motorPin, 64);  // 25% duty cycle
    } else if (speed == "50%") {
      analogWrite(motorPin, 128);
    } else if (speed == "75%") {
      analogWrite(motorPin, 192);
    } else if (speed == "100%") {
      analogWrite(motorPin, 255);
    } else {
      analogWrite(motorPin, 0);  // Stop
    }
    
    // Process direction command
    if (direction == "Right") {
      digitalWrite(steeringPin, HIGH);  // Example: HIGH for right
    } else if (direction == "Left") {
      digitalWrite(steeringPin, LOW);   // Example: LOW for left
    } else if (direction == "Straight") {
      analogWrite(steeringPin, 127);    // Center position
    }
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void connectToServer() {
  Serial.print("Connecting to server...");
  
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Connected!");
  } else {
    Serial.println("Connection failed");
  }
}