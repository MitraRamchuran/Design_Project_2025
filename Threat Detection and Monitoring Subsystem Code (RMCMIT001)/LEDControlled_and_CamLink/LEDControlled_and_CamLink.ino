/**
 * Collect images for Edge Impulse with Flash LED + Serial Control
 */
#define WIFI_SSID ""
#define WIFI_PASS ""
#define HOSTNAME "esp32cam"
#define LED_PIN 4  // Flash LED on GPIO4

#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/extra/esp32/wifi/sta.h>
#include <eloquent_esp32cam/viz/image_collection.h>

using eloq::camera;
using eloq::wifi;
using eloq::viz::collectionServer;

bool ledState = false;  // Track LED state

void setup() {
    delay(3000);
    Serial.begin(115200);
    Serial.println("\n\n=== ESP32-CAM Image Collection ===");

    // Initialize Flash LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Start OFF

    // Camera Config (keep existing)
    camera.pinout.aithinker();
    camera.brownout.disable();
    camera.resolution.face();
    camera.quality.high();

    // Initialize Camera
    while (!camera.begin().isOk()) {
        Serial.println(camera.exception.toString());
        delay(1000);
    }

    // Connect to WiFi
    while (!wifi.connect().isOk()) {
        Serial.println(wifi.exception.toString());
        delay(5000);
    }
    Serial.printf("WiFi OK (IP: %s)\n", WiFi.localIP().toString().c_str());

    // Start Server
    while (!collectionServer.begin().isOk()) {
        Serial.println(collectionServer.exception.toString());
        delay(1000);
    }
    Serial.println("Server OK");
    Serial.println("ACCESS URL: " + collectionServer.address());

    // Print Serial Commands Help
    Serial.println("\n=== Serial Commands ===");
    Serial.println("1. 'ON'  -> Turn flash LED ON");
    Serial.println("2. 'OFF' -> Turn flash LED OFF");
    Serial.println("3. 'TOGGLE' -> Toggle LED state");
}

void loop() {
    // 1. Handle Serial Commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.equalsIgnoreCase("ON")) {
            digitalWrite(LED_PIN, HIGH);
            ledState = true;
            Serial.println("LED turned ON");
        }
        else if (command.equalsIgnoreCase("OFF")) {
            digitalWrite(LED_PIN, LOW);
            ledState = false;
            Serial.println("LED turned OFF");
        }
        else if (command.equalsIgnoreCase("TOGGLE")) {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
            Serial.printf("LED toggled %s\n", ledState ? "ON" : "OFF");
        }
        else {
            Serial.println("Invalid command. Try: ON, OFF, TOGGLE");
        }
    }

    // 2. Your existing server logic runs in background
    delay(10);  // Prevents watchdog timer issues
}