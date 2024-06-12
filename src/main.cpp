#include <Arduino.h>
#include <deque>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#include <esp_task_wdt.h>  
#endif
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "configs.h"

#define WDT_TIMEOUT_S 8 // Set the WDT timeout to 8 seconds

const int statusAddress = 0;
byte status = 0;
byte statussaved = 0;

unsigned long startupTime;


const char* mqttStateTopic = "Heatpump/status";
const char* mqttControlTopic = "Heatpump/control"; //this topic is used to control the alarm activation, as well as to activate debug data for the alarm protocol - this data was extremely usefull for the development, but is probably irrelevant now
const char* lwtTopic = "Heatpump/lwt";
const char* birthMessage = "Online";
const char* lwtMessage = "Offline";
const char* debugTopic = "Baxi/debug_data"; //Topic for debug data
const char* logTopic = "Heatpump/log"; //Topic where parts of the log are published, like restart reason and some changes to the status
const char* teleTopic = "Heatpump/tele"; //Topic for the telemetry

const char* resetCause;

#ifdef ESP8266
#define LED D0
#else
#define LED 33
#endif

const int boundaryLen = 8;
const int bufferSize = 128;

std::deque<int> dataBuffer;
int insideState = 1;
int consecutiveOnes = 0;
int boundaryAge = 0;
bool cansend = false;
// Define the size of the binary array in bytes
const int binarySizeBytes = 1;  // 8 bits

bool debug = true;

WiFiClient espClient;
PubSubClient client(espClient);
bool otaInProgress = false;

unsigned long previousMillis = 0;
unsigned long previousMillistele = 0;
const unsigned long interval = 1000;
const unsigned long intervaltele = 10000;
unsigned long lastActivationTime = 0;

// client.publish(mqttStateTopic, "D", true);

void debugprint(const char *a) {
  if (debug) {
    Serial.print(a);
  }
}

void debugprint(int a) {
  if (debug) {
    Serial.print(a);
  }
}

void debugln() {
  if (debug) {
    Serial.println();
  }
}

void debugln(const char c[]) {
  if (debug) {
    Serial.println(c);
  }
}

void debugln(String c) {
  if (debug) {
    Serial.println(c);
  }
}

void debugln(const Printable& x) {
  if (debug) {
    Serial.println(x);
  }
}

void debugln(int8_t x) {
  if (debug) {
    Serial.println(x);
  }
}

String getHexV(int value) {
  char hex[3];
  snprintf(hex, sizeof(hex), "%02X", value);
  return String(hex);
}

String convertBytesToHexString(const std::deque<int>& buffer, unsigned int length) {
  String hexValue = "";
  if (buffer.size() >= length) {
    unsigned int inicio = buffer.size() - length;
    for (unsigned int i = inicio; i < buffer.size(); i++) {
      char hex[3];
      snprintf(hex, sizeof(hex), "%02X", buffer[i]);
      hexValue += hex;
    }
  }
  return hexValue;
}

void printBuffer(const std::deque<int>& buffer, unsigned int length) {
  if (buffer.size() >= length) {
    unsigned int inicio = buffer.size() - length;
    /*for (unsigned int i = inicio; i < buffer.size(); i++) {
      debugprint(buffer[i]);
    }
    debugln();*/
    String hexValue = convertBytesToHexString(buffer, length);
    //debugln(hexValue.c_str());

    if (debug) {
      client.publish(debugTopic, hexValue.c_str());
    }

    if (buffer[inicio + 2] == 0xB4) {
      client.publish("Baxi/debug_B4", hexValue.c_str());

    } else if (buffer[inicio + 2] == 0x02) {
      // looks to be the "settings" we have set? 
      // byte 6 seems to be the temperature exactly in integer
      client.publish("Heatpump/requested_temp", String(buffer[inicio + 6]).c_str());

      client.publish("Baxi/debug_02", hexValue.c_str());

    } else if (buffer[inicio + 2] == 0xB1) {
      // 64 heating, 00 not heating - almost sure que isto é
      String is_pump_heating = buffer[inicio + 10] == 0x64 ? "1" : (buffer[inicio + 10] == 0x00 ? "0" : "unknown");
      client.publish("Heatpump/is_pump_heating", is_pump_heating.c_str());
      // 51 - electric + pump,  11 pump, 00 off, 40 only eletric
      String heating_type = buffer[inicio + 21] == 0x51 ? "eletric+pump" : (buffer[inicio + 21] == 0x11 ? "pump" : (buffer[inicio + 21] == 0x0 ? "off" : (buffer[inicio + 21] == 0x40 ? "eletric" : "unknown")));
      int heating_type_int = buffer[inicio + 21] == 0x51 ? 3 : (buffer[inicio + 21] == 0x11 ? 1 : (buffer[inicio + 21] == 0x0 ? 0 : (buffer[inicio + 21] == 0x40 ? 2 : 99)));
      client.publish("Heatpump/heating_type", heating_type.c_str());

      float temp = (buffer[inicio + 16] - 30) / 2.0;
      float temp2 = (buffer[inicio + 23] - 30) / 2.0;
      float untruncated_temp = (temp + temp2) / 2.0;
      int display_temp = static_cast<int>((static_cast<int>(temp) + static_cast<int>(temp2)) / 2.0);
      client.publish("Heatpump/untruncated_temp", (String(untruncated_temp)).c_str());
      client.publish("Heatpump/display_temp", (String(display_temp)).c_str());
      client.publish("Heatpump/temp_1", (String(temp)).c_str());
      client.publish("Heatpump/temp_2", (String(temp2)).c_str());

      JsonDocument jsonDoc;
      jsonDoc["temp_1"] = temp;
      jsonDoc["temp_2"] = temp2;
      jsonDoc["display_temp"] = display_temp;
      jsonDoc["untruncated_temp"] = untruncated_temp;
      jsonDoc["is_pump_heating"] = is_pump_heating;
      jsonDoc["heating_type"] = heating_type;
      jsonDoc["heating_type_int"] = heating_type_int;
    
      String jsonStr;
      serializeJson(jsonDoc, jsonStr);
      client.publish(mqttStateTopic, jsonStr.c_str());

    } else {
      client.publish("Baxi/unknown", hexValue.c_str());
      return;
    }

  }
}

/**
 * baud rate is 600 milliseconds
 */
unsigned long timetogetto = 0UL;

void readSerialPort() {
  std::deque<int> buffer;
  bool startBytesFound = false;
  bool endBytesFound = false;
  int startByteCount = 0;
  int endByteCount = 0;
  unsigned long previousMicrosBaudRate = 0UL;
  bool sentOne = false;

  unsigned long functime = micros();
  //debugln("TGH: " + String(micros() - timetogetto));

  
  while (Serial.available() > 0) {
    // baud rate 600 means we only have the next "bit" after 1.667ms. this means we need to wait 1.667ms until we loop again
    int data = Serial.read();
    previousMicrosBaudRate = micros();

    

    if (data == 0xFE && startByteCount == 0) {
      startByteCount++;
      buffer.push_back(data);
    } else if (data == 0xAA && startByteCount == 1) {
      startByteCount++;
      buffer.push_back(data);
      startBytesFound = true;
    } else if (startBytesFound && endByteCount < 2) {
      buffer.push_back(data);

      if (data == 0x55 && endByteCount == 0) {
        endByteCount++;
      } else if (data == 0xFE && endByteCount == 1) {
        endByteCount++;
        endBytesFound = true;
      } else {
        endByteCount = 0;
      }
    } else {
      // we received a byte but we were not ready!
      debugprint("*");
    }

    if (startBytesFound && endBytesFound) {
      unsigned long processTime = millis();
      digitalWrite(LED, LOW);
      // Process the buffer message
      printBuffer(buffer, buffer.size());

      if (millis() - processTime < 1300) {
        // new message seem to take 1.436 seconds after the end of the previous message
        delay(1300 - (millis() - processTime));
      }
      sentOne = true;

      // Clear the buffer and reset flags
      buffer.clear();
      startBytesFound = false;
      endBytesFound = false;
      startByteCount = 0;
      endByteCount = 0;
      
      // go away
      digitalWrite(LED, HIGH);
      //debugln("TT: " + String(micros() - functime) + "us");

      timetogetto = micros();
      return ;
    }

    if (buffer.size() > 100) {
      // something has gone terribly wrong
      String problem = "Something has gone terribly wrong: " + convertBytesToHexString(buffer, buffer.size()); 
      client.publish(debugTopic, problem.c_str());
      debugln(problem);
    }

    // 1667 micros for baud rate 600 - esp might go faster attempting to receive the next byte, so we'll wait a little bit
    unsigned long timeSinceLastByte = micros() - previousMicrosBaudRate;
    if (Serial.available() == 0 && !(timeSinceLastByte >= 1660)) {
      debugln(timeSinceLastByte);
      delayMicroseconds(timeSinceLastByte);
    }
    previousMicrosBaudRate = micros();
  }

  if (!sentOne && buffer.size() > 0) {
    debugprint(".");
  }
  
  digitalWrite(LED, HIGH);

  debugln("TT: " + String(micros() - functime));
  timetogetto = micros();
}




void callback(char* topic, byte* payload, unsigned int length) {
  // Check if OTA update is in progress, and disable the interrupt if it is
  if (otaInProgress) {
    return;
  }
  payload[length] = '\0';
  String receivedPayload = String((char*)payload);
  
  if (strcmp(topic, mqttControlTopic) == 0) {
    if (receivedPayload == "debugon") {
      client.publish(logTopic, "Debug on!");
      debugln("Debug on!");
      debug = true;
    } else if (receivedPayload == "debugoff") {
      client.publish(logTopic, "Debug off!");
      debugln("Debug off!");
      debug = false;
    } else if (receivedPayload == "restart") {
      client.publish(logTopic, "A reiniciar...");
      debugln("Restart..");
      delay(1000);
      ESP.restart();
    }
  }
}

void setup() {
  #ifdef ESP8266
  // Enable the Watchdog Timer
  ESP.wdtEnable(WDT_TIMEOUT_S * 1000000); // Convert seconds to microseconds

  // Get reset cause
  rst_info* resetInfo = ESP.getResetInfoPtr();
  String resetCause;
  switch (resetInfo->reason) {
    case REASON_DEFAULT_RST:
      resetCause = "Power-on reset";
      break;
    case REASON_WDT_RST:
      resetCause = "Watchdog reset";
      break;
    case REASON_EXCEPTION_RST:
      resetCause = "Exception reset";
      break;
    case REASON_SOFT_WDT_RST:
      resetCause = "Software Watchdog reset";
      break;
    case REASON_SOFT_RESTART:
      resetCause = "Software restart";
      break;
    case REASON_DEEP_SLEEP_AWAKE:
      resetCause = "Deep sleep wake-up";
      break;
    case REASON_EXT_SYS_RST:
      resetCause = "External reset";
      break;
    default:
      resetCause = "Unknown reset cause";
      break;
  }
  #else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);             // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);     
    int BootReason = esp_reset_reason();
    String resetCause;
    switch (BootReason) {
      case ESP_RST_POWERON:
        resetCause = "Power-on reset";
        break;
      case ESP_RST_INT_WDT:
        resetCause = "Watchdog reset";
        break;
      case ESP_RST_TASK_WDT:
        resetCause = "Exception reset";
        break;
      case ESP_RST_WDT:
        resetCause = "Software Watchdog reset";
        break;
      case ESP_RST_SW:
        resetCause = "Software restart";
        break;
      case ESP_RST_DEEPSLEEP:
        resetCause = "Deep sleep wake-up";
        break;
      case ESP_RST_EXT:
        resetCause = "External reset";
        break;
      case ESP_RST_PANIC:
        resetCause = "Software reset due to exception/panic";
        break;
      case ESP_RST_BROWNOUT:
        resetCause = "Brownout reset";
        break;
      case ESP_RST_SDIO:
        resetCause = "Reset over SDIO";
        break;
      default:
        resetCause = "Unknown reset cause";
        break;
    }
  #endif

  startupTime = millis(); // Record the startup time in milliseconds

  // seems that I screwed up the board somehow, and the updates for FE B1 are now screwed.
  // but we do have access to the second line that contains inverted bytes.
  // invert signal is the last true
  bool invertTxBytes = true; 
  Serial.begin(600, SERIAL_8N1, SERIAL_FULL, 1, invertTxBytes);
  
  pinMode(LED, OUTPUT);
  //pinMode(dataPin, INPUT);
  
  delay(2000);

  WiFi.persistent(true);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    debugln("Connecting to WiFi...");
  }
  WiFi.setAutoReconnect(true);
  debugln("Connected to WiFi");
  debugprint("IP: ");
  debugln(WiFi.localIP());
  debugprint("RSSI: ");
  debugln(WiFi.RSSI());

  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    debugln("Connecting to MQTT...");
    if (client.connect(mqttID, mqttUser, mqttPassword, lwtTopic, 0, 1, lwtMessage)) {
      debugln("Connected to MQTT");
      client.publish(lwtTopic, birthMessage, true);
      client.subscribe(mqttControlTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      debugln(" Retrying in 5 seconds...");
      delay(5000);
    }
  }

  client.publish(logTopic, "Started");
  // Publish reset cause to logTopic
  const char* resetCauseChar = resetCause.c_str();
  client.publish(logTopic, resetCauseChar);

  client.setCallback(callback);

  // Initialize OTA
  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    debugln("OTA update started...");
  });
  
  ArduinoOTA.onEnd([]() {
    otaInProgress = false;
    debugln("\nOTA update finished!");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) debugln("OTA authentication failed");
    else if (error == OTA_BEGIN_ERROR) debugln("OTA begin failed");
    else if (error == OTA_CONNECT_ERROR) debugln("OTA connect failed");
    else if (error == OTA_RECEIVE_ERROR) debugln("OTA receive failed");
    else if (error == OTA_END_ERROR) debugln("OTA end failed");
  });

  // Start OTA
  ArduinoOTA.begin();

}



void loop() {
  if (!client.connected()) {
    debugln("Reconnecting to MQTT...");
    if (client.connect(mqttID, mqttUser, mqttPassword, lwtTopic, 0, 1, lwtMessage)) {
      debugln("Reconnected to MQTT");
      client.publish(lwtTopic, birthMessage, true);
      client.publish(logTopic, "Reconnected to MQTT");
      client.subscribe(mqttControlTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      debugln(" Retrying in 5 seconds...");
      unsigned long retryDelay = 5000;
      while (millis() - previousMillis < retryDelay) {
        // Wait for the retryDelay
        yield();
      }
      previousMillis = millis();
      return;
    }
  }
  client.loop();

  unsigned long currentMillis = millis();

  readSerialPort();

  if (currentMillis - previousMillis >= interval) {
    #ifdef ESP8266
    // Perform your action here at the specified interval
    ESP.wdtFeed();
    #else
    esp_task_wdt_reset();
    #endif
    previousMillis = currentMillis;
  }

  if (millis() - previousMillistele >= intervaltele) {
    // Perform your action here at the specified interval in intervaltele
    // send tele values
    // Calculate uptime in milliseconds
    unsigned long uptimeMillis = millis() - startupTime;
    // Convert milliseconds to HH:MM:SS format
    unsigned long seconds = uptimeMillis / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    unsigned long days = hours / 24; // Calculate days
    seconds %= 60;
    minutes %= 60;
    hours %= 24; // Ensure hours don't exceed 24

    // Create a formatted string in DDd HH:MM:SS format
    char uptimeStr[20]; // Format: DDd HH:MM:SS\0
    sprintf(uptimeStr, "%luD %02lu:%02lu:%02lu", days, hours, minutes, seconds);
    // Create a JSON object
    StaticJsonDocument<128> jsonDoc;
    // Add the uptime, RSSI and IP values to the JSON object
    jsonDoc["Uptime"] = String(uptimeStr);
    jsonDoc["IP"] = WiFi.localIP();
    jsonDoc["RSSI"] = WiFi.RSSI();
  
    // Serialize the JSON object to a string
    String jsonStr;
    serializeJson(jsonDoc, jsonStr);
    // Publish the JSON string to the MQTT teleTopic
    client.publish(teleTopic, jsonStr.c_str());

    //publishStatus(status);

    previousMillistele = millis();
  }

  // Other non-blocking tasks can go here
  // Handle OTA updates
  ArduinoOTA.handle();
}
