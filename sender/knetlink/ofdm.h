#ifndef OFDMNETLINK_H
#define OFDMNETLINK_H

#define SPI_DELAY_CNT 10

// SPI related GPIO settings
#define SPI_CLC 45 // 32+13 P8_11
#define SPI_MISO 23 // 0+23 P8_13
#define SPI_MOSI 47 // 32+15 P8_15
//#define SPI_CS 46 // 32+14 P8_16
#define SPI_CS 27 // 0+27 P8_17
//#define SPI_CS 44//P8_12
#define ADDR_BASE_0 0x44e07000
#define ADDR_BASE_1 0x4804c000

// LED related GPIO settings
#define GPIO_LED_ANODE 60
#define GPIO_LED_CATHODE 50
//#define GPIO_BUFFER_CONTROL 30
#define GPIO_BUFFER_CONTROL 51
// Qing - May 2, 2015
#define GPIO_LED_OR_PD 2 // P9_22 Choose between PD and LED
#define GPIO_H_POWER_LED 49 // P9_23 Output of high power LED
#define READ_OFFSET 0x138
#define SET_OFFSET 0x194
#define CLEAR_OFFSET 0x190
#define BIT_CLC (45-32) // 32+13 P8_11
#define BIT_MISO (23) // 0+23 P8_13
#define BIT_MOSI (47-32) // 32+15 P8_15
//#define BIT_CS (46-32) // 32+14 P8_16
#define BIT_CS (27) // 0+27 P8_17
//#define BIT_CS (44-32)//P8_12
#endif
