#ifndef CONFIG_H
#define CONFIG_H

void initPins();
void initPeripherals();

#define AmbLight    1
#define IA_Out      2
#define SCL         3
#define SDA         4
#define MISO_Perip  5
#define ledCntrlIR  19
#define SCK_Perip   21
#define MOSI_Perip  33
#define SPI_SCK     35
#define SPI_MOSI    36
#define SPI_MISO    37
#define SPI_CS      38
#define ledCntrl    40
#define Acc_CS      41
#define AQ_CS       42
#define Perip_PWR   45

#define CSI_D0      15
#define CSI_D1      16
#define CSI_D2      17
#define CSI_D3      18
#define CSI_D4      8
#define CSI_D5      9
#define CSI_D6      10
#define CSI_D7      11
#define CSI_PCLK    13
#define CSI_MCLK    14
#define CAM_PWR     39
#define CSI_VSYNC   6
#define CSI_HSYNC   7
#define TWI_SDA     48
#define TWI_SCK     47

#endif