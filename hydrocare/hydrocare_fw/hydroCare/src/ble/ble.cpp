#include "ble.h"
#include <NimBLEDevice.h>
#include <sys/time.h>
#include <time.h>
#include "measurement/measurement.h"
#include "timer/timer.h"
#include "ota/ota.h"
#include "ui/ui.h"
#include <esp_wifi.h>
#include "memory/memory.h"
#include "config/config.h"
#include "communication/communication.h"

// --- Globals ---
bool deviceConnected = false;
bool sendRgbFlag = false;
bool sendIrFlag = false;

// Session synchronization flag
extern bool sessionInitialized;  // Set to true when folder/files are ready

// External buffers from main.cpp and communication functions
extern uint16_t downsampled16x16[256];  // 16x16 downsampled RGB frame
extern uint16_t irFrame16x12[192];      // 16x12 IR thermal frame

// Characteristic Pointers
NimBLECharacteristic *pBatChar, *pLuxChar, *pPirChar, *pMmwaveChar, *pActionChar, *pVerChar, *pAmbIntChar;
NimBLECharacteristic *pRgbChar;
NimBLECharacteristic *pIrChar;

#define UUID_RGB "c2a969f6-16e9-4e08-99e7-5e6086f6a546" // Custom UUID for RGB Frame
#define UUID_IR  "d3b969f6-16e9-4e08-99e7-5e6086f6a547" // Custom UUID for IR Frame

// --- Callbacks ---
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        deviceConnected = true;
        Serial.print("[BLE] Connected to: ");
        Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        setTimer();
    }
    void onDisconnect(NimBLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("[BLE] Disconnected - Restarting Advertising");
        NimBLEDevice::startAdvertising();
        disableTimer();
    }
};
class ActionCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) override {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            String command = String(value.c_str());
            Serial.print("[BLE] Command Received: ");
            Serial.println(command);

            // --- OTA PARSER ---
            if (command.startsWith("Com;OTA")) {
                int firstSemi  = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                int thirdSemi  = command.indexOf(';', secondSemi + 1);
                int fourthSemi = command.indexOf(';', thirdSemi + 1);

                if (secondSemi != -1 && thirdSemi != -1 && fourthSemi != -1) {
                    ver = command.substring(secondSemi + 1, thirdSemi);
                    ssid = command.substring(thirdSemi + 1, fourthSemi);
                    password  = command.substring(fourthSemi + 1);

                    Serial.println("[OTA] Starting Update...");
                    Serial.printf("SSID: %s, Ver: %s\n", ssid.c_str(), ver.c_str());

                    otaUpdateAvailable = 1;
                }
            }
            // --- START PARSER (with Label) ---
            else if(command.startsWith("Com;Start")){
                // command is "Com;Start;session001"
                int firstSemi  = command.indexOf(';');          // Position of 1st ';'
                int secondSemi = command.indexOf(';', firstSemi + 1); // Position of 2nd ';'
                
                String label = "Default";
                
                // If there is a semicolon after "Start", the label starts at secondSemi + 1
                if (secondSemi != -1 && command.length() > secondSemi + 1) {
                    label = command.substring(secondSemi + 1);
                    label.trim(); // Remove any hidden newline or carriage return characters
                }

                deviceStatus = 1;
                Serial.println("\n[BLE] Logging started - Creating new session folder...");
                
                // Create a new session folder for this BLE connection
                initSessionFolder();
                // openMicAccelFile();   // DISABLED - SD task now handles append-based logging
                // initSensorDataFile(); // DISABLED - SD task now handles append-based logging
                // NOTE: RGB and IR are written per-packet in SD task, sensor/accel append to single files
                
                sessionInitialized = true;  // NOW safe to log
                Serial.println("[BLE] Session files ready for logging");
            }
            // --- STOP PARSER ---
            else if(command.startsWith("Com;Stop")){
                deviceStatus = 0;
                sessionInitialized = false;  // Reset for next session
                // RGB/IR files are already closed by SD task after each write
                Serial.println("[SD] Stop Logging. Session ended.");
            }
            // --- CONTROL PARSER ---
            else if (command.startsWith("Com;Control")) {
                int firstSemi = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                int thirdSemi = command.indexOf(';', secondSemi + 1);

                if (secondSemi != -1 && thirdSemi != -1) {
                    String type = command.substring(secondSemi + 1, thirdSemi);
                    String valStr = command.substring(thirdSemi + 1);
                    int val = valStr.toInt();

                    Serial.printf("[BLE] Control %s: %d\n", type.c_str(), val);
                    // Add hardware control logic here
                    // e.g., if (type == "IR") { ... }
                    if(type == "IR"){
                        Serial.print("IR LED status is ");
                        Serial.println(val);
                        sendIRLED(val);
                    }
                    if(type == "LED"){
                        Serial.print("Power LED brightness is ");
                        Serial.println(val);
                        sendBrightness(val);
                    }
                }
            }
            // --- RGB REQUEST PARSER ---
            else if (command.startsWith("Com;RGB")) {
                Serial.println("[BLE] Command RGB received. Queueing frame transmission...");
                sendRgbFlag = true;
            }
            // --- FRAMES REQUEST PARSER ---
            else if (command.startsWith("Com;Frames")) {
                Serial.println("[BLE] Command Frames received. Queueing RGB and IR frame transmission...");
                sendRgbFlag = true;
                sendIrFlag = true;
            }
            // --- SET TIME PARSER ---
            else if (command.startsWith("Com;SetTime")) {
                int firstSemi = command.indexOf(';');
                int secondSemi = command.indexOf(';', firstSemi + 1);
                
                if (secondSemi != -1) {
                    String timestampStr = command.substring(secondSemi + 1);
                    timestampStr.trim();
                    time_t timestamp = timestampStr.toInt();
                    
                    if (timestamp > 0) {
                        struct timeval tv;
                        tv.tv_sec = timestamp;
                        tv.tv_usec = 0;
                        settimeofday(&tv, NULL);
                        
                        // Show confirmation
                        time_t now = time(nullptr);
                        char timeStr[30];
                        strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", localtime(&now));
                        Serial.printf("[BLE] System time set to: %s\n", timeStr);
                    } else {
                        Serial.println("[BLE] Invalid timestamp");
                    }
                }
            }
        }
    }
};
String getDynamicName() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macStr[13];
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return "Urinfo_" + String(macStr);
}
void initBLE() {
    String devName = getDynamicName();
    Serial.printf("[BLE] Initializing: %s\n", devName.c_str());
    
    // 1. Initialize with the dynamic name
    NimBLEDevice::init(devName.c_str());
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // 1. Create the Firmware Version Characteristic (Read Only)
    // Use a unique UUID (e.g., UUID_VERSION) defined in your ble.h
    pVerChar = pService->createCharacteristic(UUID_VERSION, NIMBLE_PROPERTY::READ);
    
    // 2. Set the value from your config.cpp define
    pVerChar->setValue(fw_version); 

    pBatChar    = pService->createCharacteristic(UUID_BATTERY, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    pLuxChar    = pService->createCharacteristic(UUID_LUX,     NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    pPirChar    = pService->createCharacteristic(UUID_PIR,     NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    pMmwaveChar = pService->createCharacteristic(UUID_MMWAVE,  NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    
    // Internal Ambient Light (SPI Data)
    pAmbIntChar = pService->createCharacteristic(UUID_AMB_INT, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

    pActionChar = pService->createCharacteristic(UUID_ACTION, NIMBLE_PROPERTY::WRITE);
    pActionChar->setCallbacks(new ActionCallbacks());

    // Create RGB Image characteristic
    pRgbChar = pService->createCharacteristic(UUID_RGB, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);
    pIrChar = pService->createCharacteristic(UUID_IR, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

    // Initial Values
    pBatChar->setValue("0.0");
    pLuxChar->setValue("0");
    pPirChar->setValue("0");
    pMmwaveChar->setValue("0,0,0,0,0");
    pAmbIntChar->setValue("0");

    pService->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->setScanResponse(true);

    // FIX: Use devName.c_str() here instead of the hardcoded "Urinfo"
    pAdv->setName(devName.c_str()); 

    pAdv->setMinPreferred(0x06);
    pAdv->setMaxPreferred(0x12);
    pAdv->start();

    Serial.printf("[BLE] %s is advertising (FW: %s)...\n", devName.c_str(), fw_version);
}
void sendValue(NimBLECharacteristic* chr, String val) {
    if (deviceConnected && chr != nullptr) {
        chr->setValue((uint8_t*)val.c_str(), val.length());
        chr->notify();
    }
}

// Send downsampled 16x16 RGB and 16x12 IR images via BLE (called from notifyAll every 1 second)
void sendDownsampledImages() {
    if (!deviceConnected) return;
    
    // Send 16x16 downsampled RGB frame (256 pixels * 2 bytes = 512 bytes)
    if (pRgbChar != nullptr) {
        uint8_t* ptr = (uint8_t*)downsampled16x16;
        int remaining = 512;  // 16x16 pixels * 2 bytes
        int offset = 0;
        
        //Serial.println("[BLE] Sending 16x16 downsampled RGB frame...");
        
        while (remaining > 0) {
            int chunkSize = remaining > 200 ? 200 : remaining;
            uint8_t payload[202];
            
            payload[0] = offset & 0xFF;
            payload[1] = (offset >> 8) & 0xFF;
            memcpy(&payload[2], ptr + offset, chunkSize);
            
            pRgbChar->setValue(payload, chunkSize + 2);
            pRgbChar->notify();
            
            offset += chunkSize;
            remaining -= chunkSize;
            delay(5);  // Small delay between chunks
        }
    }
    
    // Send 16x12 IR frame (192 pixels * 2 bytes = 384 bytes)
    if (pIrChar != nullptr) {
        uint8_t* ptr = (uint8_t*)irFrame16x12;
        int remaining = 384;  // 16x12 pixels * 2 bytes
        int offset = 0;
        
        //Serial.println("[BLE] Sending 16x12 IR frame...");
        
        while (remaining > 0) {
            int chunkSize = remaining > 200 ? 200 : remaining;
            uint8_t payload[202];
            
            payload[0] = offset & 0xFF;
            payload[1] = (offset >> 8) & 0xFF;
            memcpy(&payload[2], ptr + offset, chunkSize);
            
            pIrChar->setValue(payload, chunkSize + 2);
            pIrChar->notify();
            
            offset += chunkSize;
            remaining -= chunkSize;
            delay(5);  // Small delay between chunks
        }
    }
    
    //Serial.println("[BLE] Image transmission complete!");
}

void notifyAll() {
    if (!deviceConnected) return;

    // 1. Trigger SPI Read from Slave before notifying

    // 2. Notify basic sensors
    sendValue(pBatChar, String(batteryPercentage, 1));
    sendValue(pLuxChar, String(ambLight, 0));
    sendValue(pPirChar, String(PIRValue));
    
    // 3. Notify SPI-sourced Ambient-I
    sendValue(pAmbIntChar, String((int)ambLight_Int));

    // 4. Notify mmWave data
    String mmwave = String((int)movingDist)    + "," +
                    String((int)movingEnergy)  + "," +
                    String((int)staticDist)    + "," +
                    String((int)staticEnergy)  + "," +
                    String((int)detectionDist);

    sendValue(pMmwaveChar, mmwave);
    
    // 5. Send downsampled 16x16 RGB and 16x12 IR frames every 1 second
    sendDownsampledImages();
}