/*
PUNKT A: Messverfahren

Im Datenblatt steht:

S.8 : "The data is stable when the sensor works after 30 seconds"
bei Stutgart wird nur 15s (WARMUPTIME_SDS_MS) gewartet.

Nur als Info:
S.4: In Querry Mode das empfohlene Zeitinterval der Datenabfrage ist grosser als 3s.

FRAGEN:
1. Warum bei Stutgart ist anderes?

TODO: WARMUPTIME_SDS_MS auf 30s vergrossen.

PUNKT B: Datenablage
TODO: Daten in zwischen Buffer schreiben. So können wir Seiten ständig aufbauen. Dann brauchen wir nach je Seiten Update nicht auf neue Daten zu warten.


PUNKT C: WiFi
TODO: Test with another ip adress (for example 192.168.1.200)
TODO: Test without hard address in router for esp32
TODO: Ereignisbasierte Update (WebSocket.h ???) und nicht jede 300ms Daten abfragen, sondern bei neu Laden sofort und dann je 1 Minute.
TODO: Graph wird nicht gebaut, da Internet nicht erreichbar ist => Menu Punkt Graph raus - oder csv Datei anbinden?
TODO:  im Netz brauche ich kein Menu Punkt Wifi Manager => entweder so mache, dass config hier auch übeschrieben werden kann (vielleicht es ist schon so - testen)
            oder dieses Menu Punkt loeschen.
TODO: Test - Netz schon ausgewählt und in Dateien geschpeichert - aber in anderen unbekanten Netz Gerät anschalten.

TODO: one Style for WebSeite: use table in Wifi Manager, ..

Brauche ich es? (Siehe unten)- Es ist unklar wie soll OnMessage in JavaScript die Daten bearbeiten, welche nicht in JSON Format sind.
Sensor Config mit Buuton 1: intensiv Mode - messen 1Minute lang, dann Standart weiter.
                           Button 2: Standart  - wie in Stutgart
                            Slider: für Einstellung der Schlafzeit
Ampel.html - oder andere Möglichkeit um im Farbe Ampel darzustellen.

WebSocket für CurrentData.html um 300ms zu vermeiden
Benutzen 2 slider für Messungdauer in Sensor Config um JSON Format zu nutzen.
Benutzen JSON für Buffer - um Messwerte für 1 Stunde zu speichern und in Graph darstellen bzw. in CSV-Datei ablegen.

Benutzung WEbSocket in CurrentData, SensorConfig - 1 Woche (15 Stunden),
Benutzung Buffer für Graph - 1 Woche (15 Stunden)
Alles Gerade ziehen, Style, Test 2 Wochen (30 Stunden)
Benutzerhandbuch 3 Tage (6 Stunden)

https://randomnerdtutorials.com/esp32-web-server-websocket-sliders/

branch master in Ordner HTML_onESP git id e8010ab1ee822f00c000

TODO: Debouncer fuer Taster, oder Schaltung anpassen
TODO: Autoreset nach 28 Tagen des Betriebes


*/
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <HardwareSerial.h>

const int ButtonPin =  5;
const int LED_RED        =  19;// the number of the LED pin
const int LED_GREEN   =  18;// the number of the LED pin
const int LED_YELLOW =  23;// the number of the LED pin

// Es muss zuerst deklariert werden, ansonsten Compiler bringt ein Error Message
enum class PmSensorCmd {
  Start,
  Stop,
  ContinuousMode
};

// ----------------------------------- BUTTON -----------------------------------------------
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button button1 = {ButtonPin, 0, false};

void IRAM_ATTR isr() {
  button1.numberKeyPresses += 1;
  button1.pressed = true;
}
// -------------------------------------------------------------------------

//---------------- SDS ----------
/*
 sds_periud_ms = active_phase_ms + sds_sleeping_phase_ms
 active_phase = warm_phase_ms + sds_reading_phase_ms

 */
#define WARMUPTIME_SDS_MS 15000
#define READINGTIME_SDS_MS 5000//5000
#define SLEEPTIME_SDS_MS 120000 //125000

unsigned sds_periud_ms;
unsigned sds_sleeping_phase_ms = SLEEPTIME_SDS_MS;
unsigned sds_active_phase_ms;
unsigned sds_reading_phase_ms = READINGTIME_SDS_MS;
unsigned sds_warm_phase_ms = WARMUPTIME_SDS_MS;
bool intensiv_mode;

#define ACTIVEPHASE (msSince(starttime) < sds_active_phase_ms)
#define READINGPHASE (msSince(starttime) > sds_warm_phase_ms)


#define SAMPLETIME_SDS_MS  1000

#define msSince(timestamp_before) (act_milli - (timestamp_before))

#define UPDATE_MIN(MIN, SAMPLE) if (SAMPLE < MIN) { MIN = SAMPLE; }
#define UPDATE_MAX(MAX, SAMPLE) if (SAMPLE > MAX) { MAX = SAMPLE; }
#define UPDATE_MIN_MAX(MIN, MAX, SAMPLE) { UPDATE_MIN(MIN, SAMPLE); UPDATE_MAX(MAX, SAMPLE);}

#define serialSDS (Serial2)


enum {
  SDS_REPLY_HDR = 10,
  SDS_REPLY_BODY = 8
} SDS_waiting_for;

String last_value_SDS_version;

float AQI10 = 11;
float AQI25 = 12;
float last_value_SDS_P10 = 32;
float last_value_SDS_P25 = 10;

unsigned long act_milli;
unsigned long starttime_SDS;
unsigned long starttime;

unsigned long SDS_error_count;

uint32_t sds_pm10_sum = 0;
uint32_t sds_pm25_sum = 0;
uint32_t sds_pm10_max = 0;
uint32_t sds_pm10_min = 20000;
uint32_t sds_pm25_max = 0;
uint32_t sds_pm25_min = 20000;
uint32_t sds_val_count = 0;

bool is_SDS_running = true;
bool start_sds_periud = false;
bool calculate = true;

void SDS_rawcmd(const uint8_t cmd_head1, const uint8_t cmd_head2, const uint8_t cmd_head3) {
  constexpr uint8_t cmd_len = 19;

  uint8_t buf[cmd_len];
  buf[0] = 0xAA;
  buf[1] = 0xB4;
  buf[2] = cmd_head1;
  buf[3] = cmd_head2;
  buf[4] = cmd_head3;
  for (unsigned i = 5; i < 15; ++i) {
    buf[i] = 0x00;
  }
  buf[15] = 0xFF;
  buf[16] = 0xFF;
  buf[17] = cmd_head1 + cmd_head2 + cmd_head3 - 2;
  buf[18] = 0xAB;
  serialSDS.write(buf, cmd_len);
}

// --------  Aus LIB -SDS011 ---------

float calcAQIpm25(float pm25) {
  float pm1 = 0;
  float pm2 = 12;
  float pm3 = 35.4;
  float pm4 = 55.4;
  float pm5 = 150.4;
  float pm6 = 250.4;
  float pm7 = 350.4;
  float pm8 = 500.4;

  float aqi1 = 0;
  float aqi2 = 50;
  float aqi3 = 100;
  float aqi4 = 150;
  float aqi5 = 200;
  float aqi6 = 300;
  float aqi7 = 400;
  float aqi8 = 500;

  float aqipm25 = 0;
  // float pm25 = pm25_int/10;

  if (pm25 >= pm1 && pm25 <= pm2) {
    aqipm25 = ((aqi2 - aqi1) / (pm2 - pm1)) * (pm25 - pm1) + aqi1;
  } else if (pm25 >= pm2 && pm25 <= pm3) {
    aqipm25 = ((aqi3 - aqi2) / (pm3 - pm2)) * (pm25 - pm2) + aqi2;
  } else if (pm25 >= pm3 && pm25 <= pm4) {
    aqipm25 = ((aqi4 - aqi3) / (pm4 - pm3)) * (pm25 - pm3) + aqi3;
  } else if (pm25 >= pm4 && pm25 <= pm5) {
    aqipm25 = ((aqi5 - aqi4) / (pm5 - pm4)) * (pm25 - pm4) + aqi4;
  } else if (pm25 >= pm5 && pm25 <= pm6) {
    aqipm25 = ((aqi6 - aqi5) / (pm6 - pm5)) * (pm25 - pm5) + aqi5;
  } else if (pm25 >= pm6 && pm25 <= pm7) {
    aqipm25 = ((aqi7 - aqi6) / (pm7 - pm6)) * (pm25 - pm6) + aqi6;
  } else if (pm25 >= pm7 && pm25 <= pm8) {
    aqipm25 = ((aqi8 - aqi7) / (pm8 - pm7)) * (pm25 - pm7) + aqi7;
  }
  return round(aqipm25);
}

float  calcAQIpm10(float pm10) {
  float pm1 = 0;
  float pm2 = 54;
  float pm3 = 154;
  float pm4 = 254;
  float pm5 = 354;
  float pm6 = 424;
  float pm7 = 504;
  float pm8 = 604;

  float aqi1 = 0;
  float aqi2 = 50;
  float aqi3 = 100;
  float aqi4 = 150;
  float aqi5 = 200;
  float aqi6 = 300;
  float aqi7 = 400;
  float aqi8 = 500;

  float aqipm10 = 0;
  // float pm10 = pm10_int/10;

  if (pm10 >= pm1 && pm10 <= pm2) {
    aqipm10 = ((aqi2 - aqi1) / (pm2 - pm1)) * (pm10 - pm1) + aqi1;
  } else if (pm10 >= pm2 && pm10 <= pm3) {
    aqipm10 = ((aqi3 - aqi2) / (pm3 - pm2)) * (pm10 - pm2) + aqi2;
  } else if (pm10 >= pm3 && pm10 <= pm4) {
    aqipm10 = ((aqi4 - aqi3) / (pm4 - pm3)) * (pm10 - pm3) + aqi3;
  } else if (pm10 >= pm4 && pm10 <= pm5) {
    aqipm10 = ((aqi5 - aqi4) / (pm5 - pm4)) * (pm10 - pm4) + aqi4;
  } else if (pm10 >= pm5 && pm10 <= pm6) {
    aqipm10 = ((aqi6 - aqi5) / (pm6 - pm5)) * (pm10 - pm5) + aqi5;
  } else if (pm10 >= pm6 && pm10 <= pm7) {
    aqipm10 = ((aqi7 - aqi6) / (pm7 - pm6)) * (pm10 - pm6) + aqi6;
  } else if (pm10 >= pm7 && pm10 <= pm8) {
    aqipm10 = ((aqi8 - aqi7) / (pm8 - pm7)) * (pm10 - pm7) + aqi7;
  }
  return round(aqipm10);
}

//-----------
enum {
  AIR_GOOD = 1,
  AIR_ACCEPTABLE = 2,
  AIR_SUBSTANDARD = 3,
  AIR_POOR ,
  AIR_BAD ,
  AIR_VERY_BAD
} AIR_CATEGORY;

//------------------ from Metriful
int interpretAQI(float AQI) {
  if (AQI < 50) {
    // Serial.println("AIR_GOOD");
    return AIR_GOOD;
  }
  else if (AQI < 100) {
    // Serial.println("AIR_ACCEPTABLE");
    return AIR_ACCEPTABLE;
  }
  else if (AQI < 150) {
    // Serial.println("AIR_SUBSTANDARD");
    return AIR_SUBSTANDARD;
  }
  else if (AQI < 200) {
    // Serial.println("AIR_POOR");
    return AIR_POOR;
  }
  else if (AQI < 300) {
    // Serial.println("AIR_BAD");
    return AIR_BAD;
  }
  else {
    // Serial.println("AIR_VERY_BAD");
    return AIR_VERY_BAD;
  }
}
// ----------------------------------------------------------------

/*****************************************************************
 * send SDS011 command (start, stop, continuous mode, version    *
 *****************************************************************/

template<typename T, std::size_t N> constexpr std::size_t array_num_elements(const T(&)[N]) {
  return N;
}

bool SDS_checksum_valid(const uint8_t (&data)[8]) {
    uint8_t checksum_is = 0;
    for (unsigned i = 0; i < 6; ++i) {
        checksum_is += data[i];
    }
    return (data[7] == 0xAB && checksum_is == data[6]);
}
bool SDS_cmd(PmSensorCmd cmd) {
  switch (cmd) {
  case PmSensorCmd::Start:
    SDS_rawcmd(0x06, 0x01, 0x01);
    break;
  case PmSensorCmd::Stop:
    SDS_rawcmd(0x06, 0x01, 0x00);
    break;
  case PmSensorCmd::ContinuousMode:
    // TODO: Check mode first before (re-)setting it
    SDS_rawcmd(0x08, 0x01, 0x00);
    SDS_rawcmd(0x02, 0x01, 0x00);
    break;
  }

  return cmd != PmSensorCmd::Stop;
}


/*****************************************************************
 * read SDS011 sensor serial and firmware date                   *
 *****************************************************************/
static String SDS_version_date() {

  if (!last_value_SDS_version.length()) {
//    debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(DBG_TXT_SDS011_VERSION_DATE));
    Serial.println("SDS011 start");
    is_SDS_running = SDS_cmd(PmSensorCmd::Start);
    delay(500);

    serialSDS.flush();
    // Query Version/Date
    SDS_rawcmd(0x07, 0x00, 0x00);
    delay(400);
    const constexpr uint8_t header_cmd_response[2] = { 0xAA, 0xC5 };
    while (serialSDS.find(header_cmd_response, sizeof(header_cmd_response))) {
      uint8_t data[8];
      unsigned r = serialSDS.readBytes(data, sizeof(data));
      if (r == sizeof(data) && data[0] == 0x07 && SDS_checksum_valid(data)) {
        char tmp[20];
        snprintf_P(tmp, sizeof(tmp), PSTR("%02d-%02d-%02d(%02x%02x)"),
          data[1], data[2], data[3], data[4], data[5]);
        last_value_SDS_version = tmp;
        break;
      }
    }
//    debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(DBG_TXT_SDS011_VERSION_DATE));
  }

  return last_value_SDS_version;
}


static void powerOnTestSensors() {

//  debug_outln_info(F("Read SDS...: "), SDS_version_date());
  Serial.println("Read SDS...: "+ SDS_version_date());
  SDS_cmd(PmSensorCmd::ContinuousMode);
  delay(100);
//  debug_outln_info(F("Stopping SDS011..."));
    Serial.println("SDS011 stopping");
  is_SDS_running = SDS_cmd(PmSensorCmd::Stop);

}


static void fetchSensorSDS(String& s) {

 if(ACTIVEPHASE) {
    if (! is_SDS_running) {
      Serial.println("SDS011 start ACTIVEPHASE: "+ String(millis()));
      is_SDS_running = SDS_cmd(PmSensorCmd::Start);
      SDS_waiting_for = SDS_REPLY_HDR;
    }
//    Serial.println("SDS011 Messung "+ String(millis())); // 20s Messung
    while (serialSDS.available() >= SDS_waiting_for) {
      const uint8_t constexpr hdr_measurement[2] = { 0xAA, 0xC0 };
      uint8_t data[8];

      switch (SDS_waiting_for) {
      case SDS_REPLY_HDR:
        if (serialSDS.find(hdr_measurement, sizeof(hdr_measurement)))
          SDS_waiting_for = SDS_REPLY_BODY;
        break;
      case SDS_REPLY_BODY:
 //       debug_outln_verbose(FPSTR(DBG_TXT_START_READING), FPSTR(SENSORS_SDS011));
        if (serialSDS.readBytes(data, sizeof(data)) == sizeof(data) && SDS_checksum_valid(data)) {
          uint32_t pm25_serial = data[0] | (data[1] << 8);
          uint32_t pm10_serial = data[2] | (data[3] << 8);

          if (READINGPHASE) {
            sds_pm10_sum += pm10_serial;
            sds_pm25_sum += pm25_serial;
            UPDATE_MIN_MAX(sds_pm10_min, sds_pm10_max, pm10_serial);
            UPDATE_MIN_MAX(sds_pm25_min, sds_pm25_max, pm25_serial);
//            debug_outln_verbose(F("PM10 (sec.) : "), String(pm10_serial / 10.0f));
//            debug_outln_verbose(F("PM2.5 (sec.): "), String(pm25_serial / 10.0f));
              Serial.println("PM10 (sec.) : " + String(pm10_serial / 10.0f) + " " + String(millis()));
              Serial.println("PM2.5 (sec.) : " + String(pm25_serial / 10.0f) + " " + String(millis()));
            sds_val_count++;
            calculate = true;
                if (sds_reading_phase_ms == READINGTIME_SDS_MS){
                    intensiv_mode = 0;
                } else {
                    intensiv_mode = 1;
                }
          }
        }
//        debug_outln_verbose(FPSTR(DBG_TXT_END_READING), FPSTR(SENSORS_SDS011));
        SDS_waiting_for = SDS_REPLY_HDR;
        break;
      }
    }
 } else {
     if (is_SDS_running) {
      is_SDS_running = SDS_cmd(PmSensorCmd::Stop);
      Serial.println("SDS011 start SLEEPPHASE " + String(millis()));
    }
//    Serial.println("SDS011 stopp");
   if (calculate) {
    last_value_SDS_P10 = -1;
    last_value_SDS_P25 = -1;
    if (sds_val_count > 2) {
      sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
      sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
      sds_val_count = sds_val_count - 2;
    }
    if (sds_val_count > 0) {
      last_value_SDS_P10 = float(sds_pm10_sum) / (sds_val_count * 10.0f);
      last_value_SDS_P25 = float(sds_pm25_sum) / (sds_val_count * 10.0f);
/*
      add_Value2Json(s, F("SDS_P1"), F("PM10:  "), last_value_SDS_P10);
      add_Value2Json(s, F("SDS_P2"), F("PM2.5: "), last_value_SDS_P25);
*/
//      debug_outln_info(FPSTR(DBG_TXT_SEP));
      AQI10 = calcAQIpm10(last_value_SDS_P10);
      AQI25 = calcAQIpm25(last_value_SDS_P25);
      Serial.println("SDS_P1->PM10 : " + String(last_value_SDS_P10) + " " + String(millis()));
      Serial.println("SDS_P2->PM2.5 : " + String(last_value_SDS_P25) + " " + String(millis()));
      Serial.println("AQI10 : " + String(AQI10) + " " + String(millis()));
      Serial.println("AQI2.5: " + String(AQI25) + " " + String(millis()));

      calculate = false;

      if (sds_val_count < 3) {
        SDS_error_count++;
        Serial.println("SDS_error_count :" + String(SDS_error_count));
      }
    } else {
      SDS_error_count++;
      Serial.println("SDS_error_count " + String(SDS_error_count));
    }
    sds_pm10_sum = 0;
    sds_pm25_sum = 0;
    sds_val_count = 0;
    sds_pm10_max = 0;
    sds_pm10_min = 20000;
    sds_pm25_max = 0;
    sds_pm25_min = 20000;

  }
 }
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// --------------------------------------------- WEB -----------------------------------------------------------------------------

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ip";
const char* PARAM_INPUT_4 = "gateway";


//Variables to save values from HTML form
String ssid;
String pass;
String ip;
String gateway;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ipPath = "/ip.txt";
const char* gatewayPath = "/gateway.txt";

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

/* const char* ssid = "FRITZ!Box 7590 XY";
const char* password = "79384766866246325338"; */

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


String get_pm10() {
     Serial.println(last_value_SDS_P10);
    return String(last_value_SDS_P10);
}

String get_pm25() {
     Serial.println(last_value_SDS_P25);
    return String(last_value_SDS_P25);
}

String get_aqi_pm25() {

  // long t = random(50,70);
     Serial.println(AQI25);
    return String(AQI25);
}

String get_aqi_pm10() {

     Serial.println(AQI10);
    return String(AQI10);
}

void serverOn(){
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
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "The content you are looking for was not found.");
  });

  server.on("/aqi_pm25", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_aqi_pm25().c_str());
  });
  server.on("/aqi_pm10", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_aqi_pm10().c_str());
  });
  server.on("/pm25", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_pm25().c_str());
  });
  server.on("/pm10", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_pm10().c_str());
  });
}

// Initialize SPIFFS
void initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }

  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- frite failed");
  }
}

// Initialize WiFi
bool initWiFi() {
  if(ssid=="" && ip==""){
    Serial.println("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());


  if (!WiFi.config(localIP, localGateway, subnet)){
    Serial.println("STA Failed to configure");
    return false;
  }
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.println("Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("Failed to connect.");
      return false;
    }
  }

  Serial.println(WiFi.localIP());
  return true;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------

void setup(){
  Serial.begin(115200);

  initSPIFFS();

  //---------------- SDS ----------
  Serial.println("SETUP " + String(millis()));
  serialSDS.begin(9600);
  serialSDS.setTimeout((4 * 12 * 1000) / 9600);
//----------------

  // Load values saved in SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  ip = readFile(SPIFFS, ipPath);
  gateway = readFile (SPIFFS, gatewayPath);
  Serial.println(ssid);
  Serial.println(pass);
  Serial.println(ip);
  Serial.println(gateway);

  if(initWiFi()) {
    serverOn();
// Start server
   server.begin();
}
  else {
    // Connect to Wi-Fi network with SSID and password
    Serial.println("Setting AP (Access Point)");
    // NULL sets an open Access Point
    WiFi.softAP("ESP-WIFI-MANAGER", NULL);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    serverOn();
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/wifimanager.html", "text/html");
    });

    server.serveStatic("/", SPIFFS, "/");

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            Serial.print("SSID set to: ");
            Serial.println(ssid);
            // Write file to save value
            writeFile(SPIFFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            Serial.print("Password set to: ");
            Serial.println(pass);
            // Write file to save value
            writeFile(SPIFFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ip = p->value().c_str();
            Serial.print("IP Address set to: ");
            Serial.println(ip);
            // Write file to save value
            writeFile(SPIFFS, ipPath, ip.c_str());
          }
          // HTTP POST gateway value
          if (p->name() == PARAM_INPUT_4) {
            gateway = p->value().c_str();
            Serial.print("Gateway set to: ");
            Serial.println(gateway);
            // Write file to save value
            writeFile(SPIFFS, gatewayPath, gateway.c_str());
          }
          //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "Done. ESP will restart, connect to your router and go to IP address: " + ip);
      delay(3000);
      ESP.restart();
    });
    server.begin();
}

    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    pinMode(button1.PIN, INPUT_PULLUP);
    attachInterrupt(button1.PIN, isr, FALLING);

  //---------------- SDS ----------
  powerOnTestSensors();
  delay(1000);
  starttime = millis();      // store the start time
  //----------------
}

#define AQI_CATEGORY_PM10 interpretAQI(calcAQIpm10(last_value_SDS_P10))
#define AQI_CATEGORY_PM25 interpretAQI(calcAQIpm25(last_value_SDS_P25))

void    loop(){
//---------------- SDS ----------
  String result_SDS;
//  Serial.println("LOOP " + String(millis()));
 sds_active_phase_ms = sds_warm_phase_ms + sds_reading_phase_ms;
 sds_periud_ms = sds_active_phase_ms + sds_sleeping_phase_ms;



  act_milli = millis();
  start_sds_periud = msSince(starttime) > sds_periud_ms;

  if (button1.pressed) {
      Serial.printf("Button 1 has been pressed %u times\n", button1.numberKeyPresses);
      button1.pressed = false;
      sds_reading_phase_ms = 15000; // wenn Taster in Reading Phase gedruckt wird, kommt SDS Error !, aber in SLEEP alles good!
  }


  if ((msSince(starttime_SDS) > SAMPLETIME_SDS_MS)){ // Takt 1s
    starttime_SDS = act_milli;

    fetchSensorSDS(result_SDS);
//    Serial.println("LEDs leuchten");  ------------------------------> TODO:jede Sekunde LEDs einsteuern ? bei Wertänderung?
     if (AQI_CATEGORY_PM10  == AIR_GOOD &&  AQI_CATEGORY_PM25  == AIR_GOOD){
     // Serial.println("LED GREEN leuchten");
     digitalWrite(LED_RED, LOW);
     digitalWrite(LED_YELLOW, LOW);
     digitalWrite(LED_GREEN, HIGH);
    } else if (AQI_CATEGORY_PM10  > AIR_ACCEPTABLE ||  AQI_CATEGORY_PM25  > AIR_ACCEPTABLE){
     // Serial.println("LED ROT leuchten");
     digitalWrite(LED_RED, HIGH);
     digitalWrite(LED_GREEN, LOW);
     digitalWrite(LED_YELLOW, LOW);
      } else{
     // Serial.println("LED GELB leuchten");
     digitalWrite(LED_RED, LOW);
     digitalWrite(LED_GREEN, LOW);
     digitalWrite(LED_YELLOW, HIGH);
    }
}
  if (start_sds_periud) { // Takt 145s - wie bei Stuttgrart
    starttime = millis();      // store the start time
    Serial.println("SDS011 start PERIUD " + String(millis()) + " / PERIUD = " + String(sds_periud_ms/1000) + "s");
    if (intensiv_mode) sds_reading_phase_ms = READINGTIME_SDS_MS; // setzt Intensiv Modus zurück.
  }
//----------------
}
