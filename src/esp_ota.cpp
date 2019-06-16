#include <ArduinoOTA.h>
#include <WiFi.h>
#include "nextioninterface.h"

#define WIFISSID "syringePump"
#define WIFIPassword "pmuPegnirys"

// Forward declarations for OTA events
void onStart(void);
void onEnd(void);
void onError(ota_error_t error);
void onProgress(unsigned int progress, unsigned int total);

uint16_t waitDelay=500;

bool startWiFi(void) {
  bool switchToX = false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFISSID, WIFIPassword);
  WiFi.setHostname("update");
  WiFi.config(IPAddress(192,168,4,127), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  Serial.printf("Connecting to AP %s\n", WIFISSID);
  updateErrorTxt("Connecting to AP");
  while ((WiFi.status() != WL_CONNECTED)) {
    yield();
    if (switchToX == true) {
      Serial.print("x");
      updateStatusTxt("OTA x");
      switchToX = false;
    }
    else {
      Serial.print("+");
      updateStatusTxt("OTA +");
      switchToX = true;
    }
    processNexMessages();
    delay(waitDelay);
  }
  if(WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    updateErrorTxt(WiFi.localIP().toString().c_str());
    char buf[32];
    snprintf(buf, sizeof(buf),"%s.local", WiFi.getHostname());
    return true;
  }
  else {
    Serial.println("Wifi connection failed.");
    updateErrorTxt("Wifi not connected");
    WiFi.mode(WIFI_MODE_NULL);
    return false;
  }
}

void runOTA() {
  if (!startWiFi()) return;

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setPassword("6628");
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.onStart(std::bind(&onStart));
  ArduinoOTA.onEnd(std::bind(&onEnd));
  ArduinoOTA.onProgress(std::bind(&onProgress, std::placeholders::_1, std::placeholders::_2));
  ArduinoOTA.onError(std::bind(&onError, std::placeholders::_1));
  ArduinoOTA.begin();
  Serial.printf("OTA service started. Hostname: %s\n", ArduinoOTA.getHostname().c_str());
  Serial.printf("Wifi status: %4X\n", WiFi.getStatusBits());

  bool switchToX = false;
  while(1) {
    yield();
    delay(waitDelay);
    if (switchToX == true) {
      Serial.print("x");
      updateStatusTxt("OTA x");
      switchToX = false;
    }
    else {
      Serial.print("+");
      updateStatusTxt("OTA +");
      switchToX = true;
    }
    processNexMessages();
    ArduinoOTA.handle();
  }
}

// OTA event handlers
void onStart(void) {
  String type;
  if (ArduinoOTA.getCommand() == U_FLASH) {
    type = "sketch";
  }
  else { // U_SPIFFS
    type = "filesystem";
  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  }

  Serial.println("Start updating " + type);
  waitDelay = 0;
};

void onEnd(void) {
  WiFi.mode(WIFI_MODE_NULL);
  Serial.println("\nEnd");
};

void onError(ota_error_t error) {
  updateErrorTxt("OTA error");
  Serial.printf("Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR) {
    Serial.println("Auth Failed");
  } else if (error == OTA_BEGIN_ERROR) {
    Serial.println("Begin Failed");
  } else if (error == OTA_CONNECT_ERROR) {
    Serial.println("Connect Failed");
  } else if (error == OTA_RECEIVE_ERROR) {
    Serial.println("Receive Failed");
  } else if (error == OTA_END_ERROR) {
    Serial.println("End Failed");
  }
};

void onProgress(unsigned int progress, unsigned int total) {
  static uint8_t prevPctProgress = 0;
  uint8_t pctProgress = (progress * 100) / total;
  if(pctProgress > prevPctProgress){
    char buf[10];
    snprintf(buf, sizeof(buf), "%u%%", pctProgress);
    Serial.println(buf);
    updateErrorTxt(buf);
    prevPctProgress = pctProgress;
  }
};
