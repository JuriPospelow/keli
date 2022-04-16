#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <HardwareSerial.h>

//---------------- SDS ----------
#define ACTIVEPHASE (msSince(starttime) < cfg::sending_intervall_ms - SLEEPTIME_SDS_MS)
#define READINGPHASE (msSince(starttime) > cfg::sending_intervall_ms - SLEEPTIME_SDS_MS - READINGTIME_SDS_MS)


#define SAMPLETIME_SDS_MS  1000
#define WARMUPTIME_SDS_MS 15000
#define READINGTIME_SDS_MS 5000
#define SLEEPTIME_SDS_MS 125000

#define msSince(timestamp_before) (act_milli - (timestamp_before))

#define UPDATE_MIN(MIN, SAMPLE) if (SAMPLE < MIN) { MIN = SAMPLE; }
#define UPDATE_MAX(MAX, SAMPLE) if (SAMPLE > MAX) { MAX = SAMPLE; }
#define UPDATE_MIN_MAX(MIN, MAX, SAMPLE) { UPDATE_MIN(MIN, SAMPLE); UPDATE_MAX(MAX, SAMPLE);}

#define serialSDS (Serial2)
#define SDS_READ 1


namespace cfg {

  unsigned sending_intervall_ms = 145000;

  bool sds_read = SDS_READ;
}

enum class PmSensorCmd {
  Start,
  Stop,
  ContinuousMode
};

enum {
  SDS_REPLY_HDR = 10,
  SDS_REPLY_BODY = 8
} SDS_waiting_for;

String last_value_SDS_version;

float AQI10 = 11;
float AQI25 = 12;
float last_value_SDS_P1 = 32;
float last_value_SDS_P2 = 34;

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
bool sds_periud = false;
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

  if (cfg::sds_read && !last_value_SDS_version.length()) {
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
      Serial.println("SDS011 start: "+ String(millis()));
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
      Serial.println("SDS011 stopp 2: " + String(millis()));
    }
//    Serial.println("SDS011 stopp");
   if (calculate) {
    last_value_SDS_P1 = -1;
    last_value_SDS_P2 = -1;
    if (sds_val_count > 2) {
      sds_pm10_sum = sds_pm10_sum - sds_pm10_min - sds_pm10_max;
      sds_pm25_sum = sds_pm25_sum - sds_pm25_min - sds_pm25_max;
      sds_val_count = sds_val_count - 2;
    }
    if (sds_val_count > 0) {
      last_value_SDS_P1 = float(sds_pm10_sum) / (sds_val_count * 10.0f);
      last_value_SDS_P2 = float(sds_pm25_sum) / (sds_val_count * 10.0f);
/*
      add_Value2Json(s, F("SDS_P1"), F("PM10:  "), last_value_SDS_P1);
      add_Value2Json(s, F("SDS_P2"), F("PM2.5: "), last_value_SDS_P2);
*/
//      debug_outln_info(FPSTR(DBG_TXT_SEP));
      AQI10 = calcAQIpm10(last_value_SDS_P1);
      AQI25 = calcAQIpm25(last_value_SDS_P2);
      Serial.println("SDS_P1->PM10 : " + String(last_value_SDS_P1) + " " + String(millis()));
      Serial.println("SDS_P2->PM2.5 : " + String(last_value_SDS_P2) + " " + String(millis()));
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

//---------------

const char* ssid = "FRITZ!Box 7590 XY";
const char* password = "79384766866246325338";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


String get_pm10() {
     Serial.println(last_value_SDS_P1);
    return String(last_value_SDS_P1);
}

String get_pm25() {
     Serial.println(last_value_SDS_P2);
    return String(last_value_SDS_P2);
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

void setup(){
      Serial.begin(115200);

  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  //---------------- SDS ----------
  Serial.println("SETUP " + String(millis()));
  serialSDS.begin(9600);
  serialSDS.setTimeout((4 * 12 * 1000) / 9600);
//----------------

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
  server.on("/aqi_pm10", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_aqi_pm10().c_str());
  });
  server.on("/pm25", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_pm25().c_str());
  });
  server.on("/pm10", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", get_pm10().c_str());
  });

  //---------------- SDS ----------
  powerOnTestSensors();
  delay(1000);
  starttime = millis();      // store the start time
  //----------------

// Start server
  server.begin();
}

void    loop(){
//---------------- SDS ----------
  String result_SDS;
//  Serial.println("LOOP " + String(millis()));
  act_milli = millis();
  sds_periud = msSince(starttime) > cfg::sending_intervall_ms;

  if ((msSince(starttime_SDS) > SAMPLETIME_SDS_MS)){ // Takt 1s
    starttime_SDS = act_milli;

    fetchSensorSDS(result_SDS);
//    Serial.println("LEDs leuchten");  ------------------------------> TODO:jede Sekunde LEDs einsteuern ? bei Wertänderung?
  }

  if (sds_periud) { // Takt 145s
    starttime = millis();      // store the start time
    Serial.println("TAKT 145s " + String(millis()));
  }
//----------------
}
