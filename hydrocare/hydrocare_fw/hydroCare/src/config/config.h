#ifndef CONFIG_H
#define CONFIG_H

void initPins();
void initPeripherals();

extern String fw_version;

#define AmbLight        1
#define Batt_LVL        2
#define PIR             3
#define Button          4
#define mmWave_Out      5
#define SPI_SCK         6
#define SPI_MOSI        7
#define SPI_MISO        8
#define SPI_CS          9
#define USB_Voltage     10
#define MOSI            11
#define SCK             12
#define MISO            13
#define FLASH_CS        15
#define Sensor_EN       16
#define mmWave_TX       17  // RX in ESP32
#define mmWave_RX       18  // TX in ESP32
#define Perip_EN        19
#define SD_CS           20
#define CE_En           47
#define Batt_EN         48

#endif