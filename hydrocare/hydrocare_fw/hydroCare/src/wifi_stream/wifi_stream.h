#ifndef wifi_stream_h
#define wifi_stream_h
#include <WiFi.h>
#include <SD.h>
#include "config/config.h"

void streamFolderToTCP(String folderName);
#endif