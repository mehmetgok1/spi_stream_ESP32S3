#include "wifi_stream.h"
#include <ble/ble.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <FS.h>
#include <SD.h>

extern String server_ip;
const String DEVICE_ID = "ESP32S3_UNIT_01"; // Unique identifier for this customer device

void streamFolderToTCP(String folderName)
{
    uint32_t taskStart = millis();
    WiFiClient client; // Use WiFiClientSecure here later for HTTPS
    HTTPClient http;

    // Construct your backend URL 
    String serverURL = "http://" + server_ip + ":8000/upload";

    // Iterate through your binary part files on the SD card
    for (int i = 0; i < 10000; i += 50)
    {
        char fileName[128];
        snprintf(fileName, sizeof(fileName), "/%s/%s_part_%u.bin", folderName.c_str(), folderName.c_str(), i);

        File file = SD.open(fileName, FILE_READ);

        // End transmission if the next expected file doesn't exist
        if (!file)
        {
            Serial.printf("[HTTP-STREAM] No more files found at index %u. Ending transmission.\n", i);
            break;
        }

        Serial.printf("[HTTP-STREAM] Preparing payload for: %s (%d bytes)\n", fileName, file.size());

        // Configure the HTTP endpoint connection
        if (http.begin(client, serverURL)) 
        {
            // Add metadata via Protocol Headers for customer/session tracking
            http.addHeader("Content-Type", "application/octet-stream");
            http.addHeader("X-Device-ID", DEVICE_ID);
            http.addHeader("X-Folder-Name", folderName);
            http.addHeader("X-Part-Index", String(i));
            
            // Stream the file directly from the SD card to the Wi-Fi client.
            // The library handles chunked streaming natively—no big RAM buffer needed!
            int httpResponseCode = http.sendRequest("POST", &file, file.size());

            if (httpResponseCode == HTTP_CODE_OK) 
            {
                Serial.printf("[HTTP-STREAM] Successfully uploaded part %u\n", i);
            } 
            else 
            {
                Serial.printf("[HTTP-STREAM] Upload failed for part %u, Error Code: %d (%s)\n", 
                              i, httpResponseCode, http.errorToString(httpResponseCode).c_str());
                
                // Optional: add "break;" or error-handling if you want to abort the session on failure
            }
            
            http.end(); // Clean up connection resources for this request
        }
        else
        {
            Serial.println("[HTTP-STREAM] Unable to connect to backend server.");
        }

        file.close();
        yield(); // Yield to core tasks to keep the ESP32 system stable
    }

    uint32_t taskDuration = millis() - taskStart;
    Serial.printf("[HTTP-STREAM] Total upload routine finished in: %u ms\n", taskDuration);
}