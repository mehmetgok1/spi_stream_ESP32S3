
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>

// WiFi credentials
String ssid = "";
String password = "";
String ver = "";

// Direct GitHub release URL for your .bin file
String firmwareUrl = "https://github.com/onuryusufcinar/urinfo/releases/download/";
String firmwareUrl_OTA = "";

bool otaUpdateAvailable = 0;

WiFiClientSecure client;

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void printUpdateError() {
  Serial.print("Update error: ");
  Serial.println(Update.getError());
  
  // Print detailed error message
  switch(Update.getError()) {
    case UPDATE_ERROR_OK:
      Serial.println("No error");
      break;
    case UPDATE_ERROR_WRITE:
      Serial.println("Flash write failed");
      break;
    case UPDATE_ERROR_ERASE:
      Serial.println("Flash erase failed");
      break;
    case UPDATE_ERROR_READ:
      Serial.println("Flash read failed");
      break;
    case UPDATE_ERROR_SPACE:
      Serial.println("Not enough space");
      break;
    case UPDATE_ERROR_SIZE:
      Serial.println("Bad size given");
      break;
    case UPDATE_ERROR_STREAM:
      Serial.println("Stream read timeout");
      break;
    case UPDATE_ERROR_MD5:
      Serial.println("MD5 check failed");
      break;
    case UPDATE_ERROR_MAGIC_BYTE:
      Serial.println("Wrong magic byte");
      break;
    case UPDATE_ERROR_ACTIVATE:
      Serial.println("Could not activate the firmware");
      break;
    case UPDATE_ERROR_NO_PARTITION:
      Serial.println("Partition could not be found");
      break;
    case UPDATE_ERROR_BAD_ARGUMENT:
      Serial.println("Bad argument");
      break;
    case UPDATE_ERROR_ABORT:
      Serial.println("Aborted");
      break;
    default:
      Serial.println("Unknown error");
      break;
  }
}
void performOTAUpdate() {
  HTTPClient http;
  client.setInsecure(); // Skip SSL verification for GitHub
  
  firmwareUrl_OTA = firmwareUrl + String(ver) + "/firmware.bin"; 
  Serial.print("Firmware URL:");
  Serial.println(firmwareUrl_OTA);
  http.begin(client, firmwareUrl_OTA);
  http.addHeader("User-Agent", "ESP32-OTA-Updater");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // Follow redirects
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    
    if (contentLength > 0) {
      Serial.println("Starting OTA update...");
      Serial.printf("Firmware size: %d bytes\n", contentLength);
      
      WiFiClient* stream = http.getStreamPtr();
      
      if (Update.begin(contentLength)) {
        Serial.println("OTA update started");
        
        size_t written = Update.writeStream(*stream);
        
        if (written == contentLength) {
          Serial.println("Written : " + String(written) + " successfully");
        } else {
          Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
        }
        
        if (Update.end()) {
          Serial.println("OTA done!");
          if (Update.isFinished()) {
            Serial.println("Update successfully completed. Rebooting...");
            delay(1000);
            ESP.restart();
          } else {
            Serial.println("Update not finished? Something went wrong!");
          }
        } else {
          Serial.println("Error Occurred. Error #: " + String(Update.getError()));
          printUpdateError();
        }
      } else {
        Serial.println("Not enough space to begin OTA");
      }
    } else {
      Serial.println("No content in response");
    }
  } else {
    Serial.printf("HTTP error: %d\n", httpCode);
  }
  
  http.end();
}
void checkForUpdate() {
  Serial.println("Checking for firmware updates...");
  
  if (WiFi.status() == WL_CONNECTED) {
    performOTAUpdate();
  } else {
    Serial.println("WiFi not connected, skipping update check");
  }
}
/*
// Function to manually trigger OTA update
void manualUpdate() {
  Serial.println("Manual update triggered");
  performOTAUpdate();
}
*/

/*/
// Function to get current firmware info
void printFirmwareInfo() {
  Serial.println("=== Firmware Information ===");
  Serial.println("Chip Model: " + String(ESP.getChipModel()));
  Serial.println("Chip Revision: " + String(ESP.getChipRevision()));
  Serial.println("Flash Size: " + String(ESP.getFlashChipSize()));
  Serial.println("Free Heap: " + String(ESP.getFreeHeap()));
  Serial.println("Sketch Size: " + String(ESP.getSketchSize()));
  Serial.println("Free Sketch Space: " + String(ESP.getFreeSketchSpace()));
}
*/