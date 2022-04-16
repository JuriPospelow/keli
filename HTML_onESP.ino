#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"


const char* ssid = "FRITZ!Box 7590 XY";
const char* password = "79384766866246325338";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


String get_aqi_pm25() {

 long t = random(15,25);
     Serial.println(t);
    return String(t);
}

void setup(){
	  Serial.begin(115200);

  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // Send web page with input fields to client
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/config.html", "text/html");
    });
    server.on("/graph.html", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/graph.html", "text/html");
    });
    server.on("/c_values.html", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/c_values.html", "text/html");
    });
    server.on("/formate.css", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/formate.css", "text/css");
    });
    server.on("/black_cat.svg", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(SPIFFS, "/black_cat.svg", "image/svg+xml");
    });

  server.on("/aqi_pm25", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_aqi_pm25().c_str());
  });

// Start server
  server.begin();
}

void 	loop(){

}
