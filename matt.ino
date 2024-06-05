#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

// WiFi network details
const char* wifiSSID = "MyRepublic 5667"; // Replace with your WiFi SSID
const char* wifiPassword = "v9nkx9cpg5"; // Replace with your WiFi password

// MQTT Broker details
const char* mqttBroker = "broker.emqx.io";
const int mqttPort = 1883;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Pin assignments
const int ultrasonicTrigPin = 2; // Trigger pin for ultrasonic sensor
const int ultrasonicEchoPin = 3; // Echo pin for ultrasonic sensor
const int ledPin = 4;  // Pin for LED

void connectToWiFi() {
    Serial.print("Connecting to WiFi");
    WiFi.begin(wifiSSID, wifiPassword);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("Connected to WiFi!");
}

void messageReceived(int messageSize) {
    String topic = mqttClient.messageTopic();
    String messageContent = "";

    while (mqttClient.available()) {
        messageContent += (char)mqttClient.read();
    }

    Serial.println("Message received: '" + messageContent + "' on topic: '" + topic + "'");

    if (topic == "SIT210/wave") {
        // Blink the LED three times for a wave
        Serial.println("Blinking LED for wave");
        for (int i = 0; i < 3; i++) {
            digitalWrite(ledPin, HIGH);
            delay(500);
            digitalWrite(ledPin, LOW);
            delay(500);
        }
    } else if (topic == "SIT210/pat") {
        // Blink the LED once for a pat
        Serial.println("Blinking LED for pat");
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
    }
}

void connectToMQTT() {
    Serial.print("Connecting to MQTT broker");
    while (!mqttClient.connect(mqttBroker, mqttPort)) {
        Serial.print(".");
        delay(5000);
    }

    Serial.println("Connected to MQTT broker!");
    mqttClient.subscribe("SIT210/wave"); // Subscribe to the wave topic
    mqttClient.subscribe("SIT210/pat"); // Subscribe to the pat topic
    mqttClient.onMessage(messageReceived); // Set the message received callback
}

void setup() {
    Serial.begin(9600);
    connectToWiFi();
    connectToMQTT();

    pinMode(ultrasonicTrigPin, OUTPUT); // Set trigger pin as output
    pinMode(ultrasonicEchoPin, INPUT); // Set echo pin as input
    pinMode(ledPin, OUTPUT); // Set LED pin as output

    // Ensure the LED is initially off
    digitalWrite(ledPin, LOW);
}

void loop() {
    static unsigned long lastWaveOrPatTime = 0;
    const unsigned long waveInterval = 2500; // Interval between detections in milliseconds

    if (!mqttClient.connected()) {
        connectToMQTT();
    }

    mqttClient.poll();

    long pulseDuration = 0, distanceCM = 0;
    digitalWrite(ultrasonicTrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicTrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicTrigPin, LOW);
    pulseDuration = pulseIn(ultrasonicEchoPin, HIGH);
    distanceCM = (pulseDuration / 2) / 29.1; // Calculate distance in cm based on pulse duration

    unsigned long currentTime = millis();
    if (currentTime - lastWaveOrPatTime > waveInterval) {
        if (distanceCM <= 10) {
            Serial.println("Pat detected");
            mqttClient.beginMessage("SIT210/pat");
            mqttClient.print("Pat detected by Archit");
            mqttClient.endMessage();
            lastWaveOrPatTime = currentTime; // Update the last wave time
        } else if (distanceCM > 10 && distanceCM <= 30) {
            Serial.println("Wave detected");
            mqttClient.beginMessage("SIT210/wave");
            mqttClient.print("Wave detected by Archit");
            mqttClient.endMessage();
            lastWaveOrPatTime = currentTime; // Update the last wave time
        }
    }
}
