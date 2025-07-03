#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// WiFi credentials
const char* ssid = "OptimusPrime";
const char* password = "pikachuface";
// TCP server
WiFiServer server(8080);
WiFiClient client;

// Car control pins (example)
const int motorPin = 25;
const int steeringPin = 26;

void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);
  pinMode(steeringPin, OUTPUT);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Start TCP server
  server.begin();
  Serial.println("TCP server started");
}

void loop() {
  // Check for new client connection
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New client connected");
    }
    return;
  }

  // Process incoming data
  while (client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    Serial.println("Received: " + command);
    
    // Parse command (format: "SPEED,DIRECTION")
    int commaIndex = command.indexOf(',');
    if (commaIndex == -1) continue;
    
    String speed = command.substring(0, commaIndex);
    String direction = command.substring(commaIndex + 1);
    
    // Process speed command
    if (speed == "25%") {
      analogWrite(motorPin, 64);  // 25% duty cycle
      Serial.println("Motor speed set to 25%");
    } else if (speed == "50%") {
      analogWrite(motorPin, 128);
      Serial.println("Motor speed set to 50%");
    } else if (speed == "75%") {
      analogWrite(motorPin, 192);
      Serial.println("Motor speed set to 75%");
    } else if (speed == "100%") {
      analogWrite(motorPin, 255);
      Serial.println("Motor speed set to 100%");
    } else {
      analogWrite(motorPin, 0);  // Stop
      Serial.println("Motor stopped");
    }
    
    // Process direction command
    if (direction == "Right") {
      digitalWrite(steeringPin, HIGH);  // Example: HIGH for right
      Serial.println("Steering set to Right");
    } else if (direction == "Left") {
      digitalWrite(steeringPin, LOW);   // Example: LOW for left
      Serial.println("Steering set to Left");
    } else if (direction == "Straight") {
      analogWrite(steeringPin, 127);    // Center position
      Serial.println("Steering set to Straight");
    }
  }
}