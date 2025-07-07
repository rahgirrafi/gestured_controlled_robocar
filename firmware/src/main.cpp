#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>

void connectToWiFi();
void connectToServer();
void moveForward(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void stopMotors();

// WiFi credentials
const char* ssid = "OptimusPrime";
const char* password = "pikachuface";

// Server configuration
const char* serverIP = "192.168.0.104";  // Python server's IP
const int serverPort = 8080;

// Motor control pins for L298N
const int ENA = 12 ;    // Left motor PWM speed control
const int IN1 = 14 ;    // Left motor direction 1
const int IN2 = 27 ;    // Left motor direction 2
const int IN3 = 26 ;    // Right motor direction 1
const int IN4 = 25 ;    // Right motor direction 2
const int ENB = 33 ;    // Right motor PWM speed control

// Reconnect settings
const unsigned long RECONNECT_INTERVAL = 5000;  // 5 seconds
unsigned long lastReconnectAttempt = 0;

WiFiClient client;

void setup() {
  Serial.begin(115200);
  
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initial motor stop
  stopMotors();
  
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
    
    // Convert speed to PWM value
    int pwmSpeed = 0;
    if (speed == "25%") pwmSpeed = 64;
    else if (speed == "50%") pwmSpeed = 128;
    else if (speed == "75%") pwmSpeed = 192;
    else if (speed == "100%") pwmSpeed = 255;
    
    // Control motors based on direction
    if (direction == "Right") {
      turnRight(pwmSpeed);
    } else if (direction == "Left") {
      turnLeft(pwmSpeed);
    } else if (direction == "Straight") {
      moveForward(pwmSpeed);
    } else {  // Stop or unknown direction
      stopMotors();
    }
  }
}

// Motor control functions
void moveForward(int speed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Right motor backward or slower
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed );  // Adjust turn sharpness here
}

void turnRight(int speed) {
  // Left motor backward or slower
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);  // Adjust turn sharpness here
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  // Both motors stop
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
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