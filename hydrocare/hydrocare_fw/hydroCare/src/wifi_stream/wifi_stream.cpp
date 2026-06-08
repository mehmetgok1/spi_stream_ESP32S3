#include "wifi_stream.h"
#include <ble/ble.h>
#include <WiFi.h>

extern String server_ip;

void streamFolderToTCP(String folderName)
{
    WiFiClient client;

    if (!client.connect(server_ip.c_str(), 8080))
    {
        Serial.println("[TCP-STREAM] Connection failed!");
        return;
    }

    // Iterate based on your known logic (0, 50, 100, 150...)
    for (int i = 0; i < 10000; i += 50)
    {
        char fileName[128];
        snprintf(fileName, sizeof(fileName), "/%s/%s_part_%u.bin", folderName.c_str(), folderName.c_str(), i);

        File file = SD.open(fileName, FILE_READ);

        // If the file doesn't exist, we assume we have reached the end of the session
        if (!file)
        {
            Serial.printf("[TCP-STREAM] No more files found at index %u. Ending transmission.\n", i);
            break;
        }

        Serial.printf("[TCP-STREAM] Streaming: %s\n", fileName);

        uint8_t buffer[1024];
        size_t bytesRead;
        while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0)
        {
            client.write(buffer, bytesRead);
            
            // Yield to background tasks to prevent watchdog resets during large file transfers
            yield(); 
        }

        file.close();
    }

    client.stop();
    Serial.println("[TCP-STREAM] Transmission complete.");
}