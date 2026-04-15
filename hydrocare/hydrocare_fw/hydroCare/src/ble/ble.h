#pragma once

// --- Service UUID ---
#define SERVICE_UUID        "11111111-1111-1111-1111-111111111110"

// --- Characteristic UUIDs ---
#define UUID_BATTERY        "11111111-1111-1111-2222-111111111112"
#define UUID_LUX            "11111111-1111-1111-2222-111111111113"
#define UUID_PIR            "11111111-1111-1111-2222-111111111114"
#define UUID_MMWAVE         "11111111-1111-1111-2222-111111111115"
#define UUID_ACTION         "11111111-1111-1111-2222-111111111116"
#define UUID_VERSION        "11111111-1111-1111-2222-111111111117"

#define UUID_AMB_INT "11111111-1111-1111-2222-111111111118"

extern bool deviceConnected;

// --- Function Declarations ---
void initBLE();
void notifyAll();
void processBLETasks();