#include "SPIFFS.h"
#include "FS.h"
#include "WiFi.h"

// Complements of:
// https://internetofhomethings.com/homethings/?p=631
String info() {
  String result;

  float flashChipSize = (float)ESP.getFlashChipSize() / 1024.0 / 1024.0;
  float flashFreq = (float)ESP.getFlashChipSpeed() / 1000.0 / 1000.0;
  FlashMode_t ideMode = ESP.getFlashChipMode();

  char buff[64];
  result = "Hardware: \n";
  result = result + "    CPU frequency: " + ESP.getCpuFreqMHz() + " MHz\n";
  snprintf(buff, sizeof(buff), "%08X", ESP.getChipRevision());
  result = result + "    Chip rev. " + buff + "\n";
  result = result + "    MAC: ";

  WiFi.mode(WIFI_MODE_STA);
  result = result + WiFi.macAddress() + "\n";
//  snprintf(buff, sizeof(buff), "%02X:%02X:%02X:%02X:%02X:%02X", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
//  result = result + buff + "\n";
  result = result + "    Signal strength: ";
  result = result + WiFi.RSSI() + "dBm\n";
  result = result + "    WiFI status: ";
  snprintf(buff, sizeof(buff), "%u", WiFi.status());
  result = result + buff + "\n";
  result = result + "    Flash chip ID: ";
  snprintf(buff, sizeof(buff), "%08X", ESP.getEfuseMac());
  result = result + buff + "\n";
  result = result + "    Flash size (SDK): " + flashChipSize + " MB\n";
  result = result + "    Flash frequency: " + flashFreq + " MHz\n";
  result = result + "    Flash write mode: " + (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN") + "\n";

  result += "\nFirmware: \n";
  result = result + "    SDK version: " + ESP.getSdkVersion() + "\n";

  if(SPIFFS.begin()){
    result = result +
    result += "\nSPIFFS:\n";
    result = result + "    Total size (bytes): " + SPIFFS.totalBytes() + "\n";
    result = result + "    Used space (bytes): " + SPIFFS.usedBytes() + "\n";
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    result += "\n    Files:\n";
    while(file){
      result = result + "    " + file.name();
      result = result + "    - size: " + file.size() + "\n";
      file = root.openNextFile();
    }
  }
  SPIFFS.end();

  return result;
}
