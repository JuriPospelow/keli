#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"


const char* ssid = "FRITZ!Box 7590 XY";
const char* password = "79384766866246325338";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get">
    input1: <input type="text" name="input1">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    input2: <input type="text" name="input2">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    input3: <input type="text" name="input3">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";
/*
String message;

void addTop(String &message)
{
  message =  F(("<!DOCTYPE html>\n"
               "<html lang='en'>\n"
               "<head>\n"
			   "<meta charset="utf-8">\n"
			   "<meta name="viewport" content="width=device-width, initial-scale=1.0">\n"
               "<title>" Fresh Air "</title>\n"
               "</head>\n"));
  message += F("<body>\n");
  message += F("<header>\n<p>Fine Nose</p>\n"
               "<p>Now you can have a fresh air!</p>\n");
}


void addBottom(String &message) {
  message += F("<footer>\n<p>");                 // The footer will display the uptime, the IP the version of the sketch and the compile date/time
  message += F("<a href="kontakt.html">Kontakt</a>");
  message += F("<a href="kontakt.html#impressum">Impressum</a>");
  message += F("p>© 2022 by JPO</p>"</p>\n</footer>\n</body>\n</html>");
  // server.send(200, "text/html", message);
}

 */
// HTML web page to handle 3 input fields (input1, input2, input3)
const char*  home(){
/*
  addTop(message);

  message += F("<article>\n"
               "<h2>Homepage</h2>\n"                                                   // here you write your html code for your homepage. Let's give some examples...
               "<p>This is an example for a webserver on your ESP8266. "
               "Values are getting updated with Fetch API/JavaScript and JSON.</p>\n"
               "</article>\n");

  message += F("<article>\n"
               "<h2>Values (with update)</h2>\n");
  message += F("<p>Internal Voltage measured by ESP: <span id='internalVcc'>");        // example how to show values on the webserver
  message += ESP.getVcc();
  message += F("</span>mV</p>\n");

  message += F("<p>Button 1: <span id='button1'>");                                    // example how to show values on the webserver
  message += digitalRead(BUTTON1_PIN);
  message += F("</span></p>\n");

  message += F("<p>Output 1: <span id='output1'>");                                    // example 3
  message += digitalRead(OUTPUT1_PIN);
  message += F("</span></p>\n");

  message += F("<p>Output 2: <span id='output2'>");                                    // example 4
  message += digitalRead(OUTPUT2_PIN);
  message += F("</span></p>\n"
               "</article>\n");

  message += F("<article>\n"
               "<h2>Switch</h2>\n"                                                     // example how to switch/toggle an output
               "<p>Example how to switch/toggle outputs, or to initiate actions. The buttons are 'fake' buttons and only styled by CSS. Click to toggle the output.</p>\n"
               "<p class='off'><a href='c.php?toggle=1' target='i'>Output 1</a></p>\n"
               "<p class='off'><a href='c.php?toggle=2' target='i'>Output 2</a></p>\n"
               "<iframe name='i' style='display:none' ></iframe>\n"                    // hack to keep the button press in the window
               "</article>\n");

  addBottom(message);
  return message; */
  return index_html;
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
      request->send(SPIFFS, "/index.html");
    });

// Start server
  server.begin();
}

void 	loop(){

}
