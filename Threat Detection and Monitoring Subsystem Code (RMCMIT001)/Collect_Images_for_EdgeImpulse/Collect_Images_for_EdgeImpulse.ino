/**
 * Collect images for Edge Impulse image
 * classification / object detection
 *
 * BE SURE TO SET "TOOLS > CORE DEBUG LEVEL = INFO"
 * to turn on debug messages
 */

// if you define WIFI_SSID and WIFI_PASS before importing the library, 
// you can call connect() instead of connect(ssid, pass)
//
// If you set HOSTNAME and your router supports mDNS, you can access
// the camera at http://{HOSTNAME}.local

#define WIFI_SSID ""
#define WIFI_PASS ""
#define HOSTNAME "esp32cam"

#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/extra/esp32/wifi/sta.h>
#include <eloquent_esp32cam/viz/image_collection.h>

using eloq::camera;
using eloq::wifi;
using eloq::viz::collectionServer;


void setup() {
    delay(3000);
    Serial.begin(115200);
    //deepseek code
    Serial.println("Attempting to connect to WiFi...");
Serial.print("SSID: ");
Serial.println(WIFI_SSID);
Serial.print("Password: ");
Serial.println(WIFI_PASS);  // Remove this line after testing if security is a concern
    Serial.println("___IMAGE COLLECTION SERVER___");

    // camera settings
    // replace with your own model!
    camera.pinout.aithinker();
    camera.brownout.disable();
    // Edge Impulse models work on square images
    // face resolution is 240x240
    camera.resolution.face();
    camera.quality.high();

    // init camera
    while (!camera.begin().isOk())
        Serial.println(camera.exception.toString());

    // connect to WiFi
    while (!wifi.connect().isOk())
      Serial.println(wifi.exception.toString());

    
    //added code begin
        // Connect to WiFi (uses WIFI_SSID/WIFI_PASS defined above)
    Serial.println("Connecting to WiFi...");
    while (!wifi.connect().isOk()) {
        Serial.println(wifi.exception.toString());
        delay(1000);
    }
    //added code end

    // init face detection http server
    while (!collectionServer.begin().isOk())
        Serial.println(collectionServer.exception.toString());

    Serial.println("Camera OK");
    Serial.println("WiFi OK");
    Serial.println("Image Collection Server OK");
    Serial.println(collectionServer.address());
//deepseek code
}


void loop() {
    // server runs in a separate thread, no need to do anything here
}